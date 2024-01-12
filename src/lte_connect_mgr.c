/**
 * @brief: 	lte_connect_mgr.c - Connection manager for the Nordic LTE modem
 *
 * @notes: 	Runs in the context of others (mainly the modem callback functions). 
 *          In future could have a simple run to completion workerQ process but not needed now. 
 * 
 * 			Copyright (c) 2022 Reliance Foundry Co. Ltd.
 *
 */

#include <zephyr.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/reboot.h>

#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>

#include <modem/modem_info.h>
#include <nrf_modem.h>
#include <subsys/modem.h>
#include <modem/sms.h>

#include <net/aws_iot.h>    // should be able to remove this when cloud.c i/f function called


#include <logging/log.h>
LOG_MODULE_REGISTER(lte_connect);

#include "lte_connect_mgr.h"
#include "cloud.h"

// Max count that LTE link can be down can be tolerated for until a system reset is initiated
#define	LTE_MAX_LINK_DOWN	4

#define LEN_SER_NUM 		8 					// Length of serial number that is the suffix of the SMS REBOOT command

#define	REBOOT_DELAY_MS		120000				// number of milliseconds to delay before rebooting system after SMS reboot command received. 
												// Setting to 2 minutes to ensure acknowledgement is sent/received to/by network

// timer block to delay the reboot of system after special SMS reboot command 
static struct k_timer reboot_delay_timer;

// track the various states of the modem, add more as required. For now, PDP Context up or down
enum modem_state {
    MODEM_STARTUP,
    MODEM_PDP_DOWN,
    MODEM_PDP_UP
};

// semaphore to block on during startup until we get a connection
K_SEM_DEFINE(lte_connected, 0, 1);

// control block holding all the details of the lte_connection manager
struct modem_control_block {
    enum modem_state state;

    // Statistics on network registrations/connections (used to debug field issues)
	int scan_cnt;		// count of scanning events
	int lost_reg_cnt;	// count of lost registration (must be done inside state machine to ensure accuracy)
	int home_reg_cnt;	// count of Home network registration
	int roam_reg_cnt;	// count of Roaming network registration
	int cell_chg_cnt;	// count of cell changes
	int	pdp_context_up;	// count of PDP context activations
	int	pdp_context_down;	// count of PDP contexts de-activations
	int offline_cnt;	// count of functional mode changes to POWER DOWN (ie: cfun=4) 
	int link_down_cnt;	// count of number of times AWS client reports it's app layer connectivity down
	int	link_down_total;	// all time count of total times app layer connectivity down
	bool app_connectivity_up;	// flag to track if our application ever obtains connectivity

	// TODO - store current staus of connectiom for gstatus command in Shell

};


// forward references
static  char * state_to_string(enum  modem_state state);


// create storage for modem control block to hold all the data associated with this module
static struct modem_control_block modem_cblk;


		/*
		 *	Global interface functions to be called from other modules
		 */


/** 
* @brief   Global interface function to check the LTE connection status 
*
* @param    void
*
* @return   bool - true means LTE PDP Context is up, false means LTE PDP context down
*
* @note     Meant to be called from the cloud module to check the status of the LTE modem link
*/
bool lte_check_pdp_context(void)
{
    if (modem_cblk.state == MODEM_PDP_UP)
        return true;
    else
        return false;

}

/** 
* @brief   Global interface function to report unexpected lack of cloud connectivity (is it locked up or bad state?) 
*
* @param    bool	app_up: flag to indicate of application layer connectivity is up (true) OR
*							down (false) 
*
* @return   nothing
*
* @note     Meant to be called from the AWS cloud module (or other modules) to signal if the application
*			layer connectivity is up or down. 
*
*			Due to the number of devices in the field that develop cloud connectivity issues over
*			and never recover, this function will REBOOT if app connnectivity is down after N successive calls. 
* 
*/
void lte_application_conn_up(bool app_up)
{
	int err;
	enum lte_lc_func_mode mode;

	if (app_up)
	{

		/*
		 * This is part of the recovery logic for field issues. In order to prevent this reboot
		 * logic from rebooting during certification testing, we keep track if we ever achieved 
		 * AWS connectivity. If yes then we can assume we are in live field operation so enable
		 * the reboot logic
		 */
		modem_cblk.app_connectivity_up = true;

		/* we did connect to AWS so reset the counter to track if we should reset after N events
		*/
		modem_cblk.link_down_cnt = 0;
	}

	else 
	{	
		// Ok, so the application layer connectivity is down, take recovery action
		
		// First check to the ensure the modem is in the Operational mode
		err = lte_lc_func_mode_get(&mode);

		if (err)
		{
			// abort and reset system on error
			LOG_ERR("Can't get functional modem from modem");
			// abort here
		}

		else
		{
		// we could do a lot of nasty things here if the modem isn't in NORMAL mode
		// but that could be vcery disruptive for different types of testing ect. 
		// For right now to resolve field issues, if it's in POWER_DOWN mode, then 
		// force back to NORMAL (or reset/abort). 
		// TODO - once we understand the field issue better (ie: is it really getting into 
		// POWER_DOWN mode?), then we can take a better action
		// update - testing/logs from field show that modem is not down. multiple DNS requests
		//
		// TODO - clean up after field testing
			if (mode == LTE_LC_FUNC_MODE_OFFLINE)
			{

				// increment statistics
				modem_cblk.offline_cnt++;

				LOG_ERR("Modem is in POWER_DOWN mode, pushing back to NORMAL");
				err = lte_lc_func_mode_set(LTE_LC_FUNC_MODE_NORMAL);
				if (err)
					LOG_ERR("Modem failed, rebooting");

					// add assert here

			}
		}

		// increment counters that track application layer link down events
		modem_cblk.link_down_cnt++;
		modem_cblk.link_down_total++;	// including the 'all time count' which is useful for statistics reporting


		LOG_WRN("AWS connection down counter: %d", modem_cblk.link_down_cnt);

		// Only enable reboot logic if had been connected to AWS in the past. This prevents reboots in situations like bench testing or certifications
	 	if (modem_cblk.app_connectivity_up == true)

		{
			if (modem_cblk.link_down_cnt > LTE_MAX_LINK_DOWN)
			{
				LOG_ERR("AWS connection down for %d occurances, rebooting", modem_cblk.link_down_cnt);
				sys_reboot(SYS_REBOOT_COLD);
			}
		}
	}	
}

/** 
* @brief   Global interface function to display network statistics to the UI Shell 
*
* @param    void
*
* @return   nothing
*
* @note     Meant to be called from the app_shell process/service
*/
void lte_stats_print(void)
{
	int	rsrp;		// RSRP value in dBm
	char *current_state_str;	// current PDP contrxt state

	printk("\nLTE network connection statistics:\n");
	printk("Registrations - Home: %d,     Roaming: %d, Lost: %d\n",
	 modem_cblk.home_reg_cnt, modem_cblk.roam_reg_cnt, modem_cblk.lost_reg_cnt);
	printk("Searching attempts:   %d, Cell change: %d, Offline mode: %d\n",
	 modem_cblk.scan_cnt, modem_cblk.cell_chg_cnt, modem_cblk.offline_cnt);
	printk("PDP Context Activations: %d, Deactivations: %d\n", modem_cblk.pdp_context_up, modem_cblk.pdp_context_down);
	printk("Total AWS session down events detected by cloud module: %d\n", modem_cblk.link_down_total);
	printk("Transient AWS session down events %d\n", modem_cblk.link_down_cnt);

	printf("Reboot on loss of AWS connectivity enabled: %d\n", modem_cblk.app_connectivity_up);

	rsrp = modem_get_rsrp_dbm_now();
	printk("RSRP current value: %d (dBm)\n", rsrp);

	current_state_str = state_to_string(modem_cblk.state);
	printk("PDP context state: %s\n", current_state_str);

}

/** 
* @brief   Global interface function to clear the network statistics 
*
* @param    void
*
* @return   nothing
*
* @note     Meant to be called from the app_shell process/service OR during initialization
*/
void lte_stats_clear(void)
{
	modem_cblk.scan_cnt = 0;
	modem_cblk.lost_reg_cnt = 0;
	modem_cblk.home_reg_cnt = 0;
	modem_cblk.roam_reg_cnt = 0;
	modem_cblk.cell_chg_cnt = 0;
	modem_cblk.pdp_context_up = 0;
	modem_cblk.pdp_context_down = 0;
	modem_cblk.offline_cnt = 0;
	modem_cblk.link_down_cnt = 0;
	modem_cblk.link_down_total = 0;
}

		/*
		 *	Utility functions for the LTE Connection manager module (string formatting routines for logging)
		 */


/** 
* @brief    Utility function to map the State to a String (used for logging state machine) 
*
* @param    state - state to map into a string
*
* @return   pointer to a string that represents the State
*
* @note     
*/
static  char * state_to_string(enum  modem_state state) 
{
	char * stringp;

	switch(state)
	{
		case MODEM_STARTUP:
		stringp = "Startup";
		break;

		case MODEM_PDP_DOWN:
		stringp = "PDP_Down";
		break;

		case MODEM_PDP_UP:
		stringp = "PDP_Up";
		break;

		default:
		// abort here
		stringp = "Invalid state";
		break;
	}
	return (stringp);
}


		/*
		 *	Internal Event State Machine Processing functions (see state diagram)
		 */


/** 
* @brief    process PDP conext changes in the STARTUP state
*
* @param    pflag to indicate new PDP state 
*
* @return   nothing
*
* @note     
*/
void proc_startup(bool pdp_context_flag)
{

	if (pdp_context_flag)
	{
		modem_cblk.state = MODEM_PDP_UP;
		modem_cblk.pdp_context_up++;
		k_sem_give(&lte_connected); // because we are in the start-up state, unblock and let the system start-up
	}

	else
	{
		modem_cblk.state = MODEM_STARTUP;	// stay in start-up state
		modem_cblk.pdp_context_down++;		
	}

}

/** 
* @brief    process PDP conext changes in the PDP context down state
*
* @param    pflag to indicate new PDP state  
*
* @return   nothing
*
* @note     
*/
void proc_pdp_down(bool pdp_context_flag)
{
	if (pdp_context_flag)
	{
		modem_cblk.state = MODEM_PDP_UP;
		modem_cblk.pdp_context_up++;
     	// tell cloud module we have active PDP context to cloud so it can reconnect
        lte_connection_change(true);
	}

	else
	{
		// another PDP context down message so that's weird / unlikely but ok
		// no need to change state but count the stat
		modem_cblk.pdp_context_down++;		
	}

}

/** 
* @brief    process PDP conext changes in the PDP context up state
*
* @param    flag to indicate new PDP state  
*
* @return   nothing
*
* @note     
*/
void proc_pdp_up(bool pdp_context_flag)
{
	if (pdp_context_flag)
	{
		// weird to get another pdp context up message when in this state but ok, count it but that's all
		modem_cblk.pdp_context_up++;
	}

	else
	{
		// PDP context has dropped, count it, change state and signal the cloud module
		modem_cblk.pdp_context_down++;
		modem_cblk.state = MODEM_PDP_DOWN;

		lte_connection_change(false);		
	}

}

/** 
* @brief    process registration messages from the modem sub-system
*
* @param    pointer to the event structure 
*
* @return   nothing
*
* @note     TODO - 
*/
void proc_registration_msgs(const struct lte_lc_evt *const evt)
{

	switch(evt->nw_reg_status) {

	case LTE_LC_NW_REG_NOT_REGISTERED:
		modem_cblk.lost_reg_cnt++;			// lost registration
		LOG_DBG("Network registration lost");
		break;


	case LTE_LC_NW_REG_REGISTERED_HOME:
		LOG_DBG("Network registration on Home network");
		// do stats
		modem_cblk.home_reg_cnt++;

        // we could have transitioned thru not-reg state so unblock and let the system start-up
        // In order to handle the blocking start-up state, give semaphore
		// this is a bit weird to keep giving the semphore but it shouldn't matter
//		k_sem_give(&lte_connected); 
		break;

	case LTE_LC_NW_REG_SEARCHING:

		LOG_DBG("Searching for network");

		// do stats on searching and then fall thru
		modem_cblk.scan_cnt++;

	case LTE_LC_NW_REG_REGISTRATION_DENIED:
	case LTE_LC_NW_REG_UNKNOWN:

		// do a lost registation statistic...but better to add a stat on denied
		modem_cblk.lost_reg_cnt++;
		break;

	case LTE_LC_NW_REG_REGISTERED_ROAMING:
		LOG_DBG("Network registration on Roaming network");
		
		// update stat on roaming 
		modem_cblk.roam_reg_cnt++;

		break;

	case LTE_LC_NW_REG_REGISTERED_EMERGENCY:
	case LTE_LC_NW_REG_UICC_FAIL:

   		break;
    
    default:

        break;      // should abort here, invalid registration event from nrf code

	}
}

/**
* @brief    LTE process PDP Connection changes
*           
* @param    flag to indicate if PDP context is up.
*
* @return   nothing
*
* @note     Called from nrf modem monitoring service on pdp context changes on the network. 
*			
*/
static void lte_proc_pdp_status(bool pdp_context_flag)
{

	// for logging, remember the current/new states
	enum  modem_state current_state, new_state;

	// for logging, declare pointers the various strings
	char *event_str, *current_state_str, *new_state_str; 

	current_state = modem_cblk.state; // save current/previous state

    // determine the current state and process event
    switch(modem_cblk.state) {

    case MODEM_STARTUP:
        proc_startup(pdp_context_flag);
        break;
        
    case MODEM_PDP_DOWN:
        proc_pdp_down(pdp_context_flag);
        break;

    case MODEM_PDP_UP:
        proc_pdp_up(pdp_context_flag);
        break;

    default: 
        break;      // should abort here as it's invalid state (coding error)
    }

	new_state = modem_cblk.state;	// save new state

	// unified logging for all states/events
	current_state_str = state_to_string(current_state);
	new_state_str = state_to_string(new_state);
	if (pdp_context_flag)
		event_str = "UP";
	else
		event_str = "DOWN";
	
	LOG_DBG("PDP context change to %s: Current state: %s, New State: %s", event_str, current_state_str, new_state_str);
	
}

/** 
* @brief  Event handler from the nrf modem sub-system. Called on change of modem status   
*
* @param    pointer to the event structure which is a union of different structures depending on the event type
*
* @return   nothing
*
* @note     TODO - 
*/
static void lte_handler(const struct lte_lc_evt *const evt)
{
	switch (evt->type) {
	
	case LTE_LC_EVT_NW_REG_STATUS:
		LOG_INF("Modem registration status change");
		proc_registration_msgs(evt);
		break;

	case LTE_LC_EVT_PSM_UPDATE:
		// note format in Hex (not Dec) but binary would be more helpful but hex is good enough
		LOG_INF("PSM parameter update (in HEX): TAU (T3412): %x, Active time (T3324): %x",
			evt->psm_cfg.tau, evt->psm_cfg.active_time);
		break;

	case LTE_LC_EVT_EDRX_UPDATE: {
		char log_buf[60];
		ssize_t len;

		len = snprintf(log_buf, sizeof(log_buf),
			       "eDRX parameter update: eDRX: %f, PTW: %f",
			       evt->edrx_cfg.edrx, evt->edrx_cfg.ptw);
		if (len > 0) {
			LOG_INF("%s", log_buf);
		}
		break;
	}
	case LTE_LC_EVT_RRC_UPDATE:
		LOG_INF("RRC mode: %s",
			evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ?
			"Connected" : "Idle");
		break;

	case LTE_LC_EVT_CELL_UPDATE:
		LOG_INF("LTE cell changed: Cell ID: %d, Tracking area: %d", evt->cell.id, evt->cell.tac);
		
		// do stat on cell change
		modem_cblk.cell_chg_cnt++;
		break;
		
	case LTE_LC_EVT_LTE_MODE_UPDATE:
	{
		char *mode;
		switch (evt->lte_mode)
		{
			case LTE_LC_LTE_MODE_LTEM:
				mode = "LTE_LC_LTE_MODE_LTEM";
				break;
			case LTE_LC_LTE_MODE_NBIOT:
				mode = "LTE_LC_LTE_MODE_NBIOT";
				break;
			default:
			case LTE_LC_LTE_MODE_NONE:
				mode = "LTE_LC_LTE_MODE_NONE";
				break;
		}
		LOG_INF("LTE mode update: lte_mode = %s", mode);
		break;
	}

	case LTE_LC_EVT_MODEM_SLEEP_ENTER:
		LOG_DBG("Modem entering sleep");
		break;

	case LTE_LC_EVT_MODEM_SLEEP_EXIT:
		LOG_DBG("Modem exited sleep");
		break;

	case LTE_LC_EVT_MODEM_SLEEP_EXIT_PRE_WARNING:
		LOG_DBG("Modem sleep exit pre-warning");
		break;

	default:
		LOG_ERR("LTE handler, unknown event type: evt.type = %d", evt->type);
		break;
	}
}

/** 
* @brief	SMS listener function - listens for SMS messages from modem
			(msg format: REBOOT-12 digits of serial Number. eg: REBOOT-602571587412)
*
* @param	datap - pointer to SMS data
*			void - context ID 
*		
* @return	Reboots System if special SMS command received. 
*
* @note		This callback function is installed during start-up phase of the system.
* @note		A delay takes place before rebooting the system to allow the modem to acknowledge the SMS message to the SMSC. 
*				failing to ack the message will cause a repeated reboots because the SMSC will send the message if no ack received. 
*
* @todo 	Another additional safe guard would be to have an 'SMS arm reboot' command that must be sent before processing SMS-reboot commands
*			this would allow the operations personal to control/add an additional delays  
*/
static void lte_sms_listener(struct sms_data *const datap, void *context)
{
	LOG_DBG("SMS message received: %s", datap->payload);
	
	char cmp_msg[20] = "REBOOT-";
	char serial_num[50];
	  
	strcpy(serial_num, device_config_get_serial_number());

	// Combining the string REBOOT- First 8 Serial Number Character
	strncat(cmp_msg, serial_num, LEN_SER_NUM);

	// Matching the SMS to STRING REBOOT-(First 12 characters of device serial number)
	if(strcmp(cmp_msg, datap->payload) == 0)
	{
		LOG_DBG("Message Verified and System is rebooting after delay period");

		// start timer for system to reboot to allow SMS acknoledgement to be sent to the network. Failing to do so will cause continous reboots every few minutes 
		k_timer_start(&reboot_delay_timer, K_MSEC(REBOOT_DELAY_MS), K_MSEC(0));
	}
	
}

/** 
* @brief	Reboot timer expiry function 
*
* @param	pointer to timer structure - not used 
*		
* @return	Doesn't return, it reboots the system 
*
* @note
*/
static void lte_reboot_tmr_exp(struct k_timer *time)
{
	sys_reboot(0);
}


/** 
* @brief	LTE connection manager initialization - called during system start-up
*
* @param	none 
*		
* @return	err - error code if error occurred    
*/
int lte_connect_init(void)
{
    int err;

    // initialize state
    modem_cblk.state = MODEM_STARTUP;

	// ensure statistics on network are cleared
	lte_stats_clear();
	modem_cblk.app_connectivity_up = false;			// initialize to not connected to AWS, We can't do this in the reset stats function as a user can reset the stats during runtime. 


    // reset modem but not a factory reset. Note: I'm not sure why we do this
    modem_factory_reset(false);

	/* 
    * Lookup/determine and initialize the APN and PDN family to be
    * used for this SIM card. The modem is initalized with this APN. 
	* 
	* Also setup the callback  function to handle changes in PDP context. 
	*/
 	modem_init_apn_pdp_context(lte_proc_pdp_status);

	// initialize the link controller and return
	lte_lc_init_and_connect_async(lte_handler);

    // take sem and block until lte connection made and PDP context is activated
	k_sem_take(&lte_connected, K_FOREVER);

	// Timer for SMS reboot delay	
	k_timer_init(&reboot_delay_timer, lte_reboot_tmr_exp, NULL);

	// Register SMS message listener - installs callback for SMS received messages
	err = sms_register_listener(lte_sms_listener, NULL);
	if (err) {
		LOG_ERR("Error code from SMS register callback function %d", err);
		return err;
	}

    return 0;
}
