/**
 * @brief: 	modem.c - Modem utility functions
 *
 * @notes: 	Runs in the context of others (mainly the start-up thread 
 * 
 * 			Copyright (c) 2023 Reliance Foundry Co. Ltd.
 *
 */


#include <string.h>

/* Modem */
#include <device.h>
#include <modem/lte_lc.h>
#include <modem/modem_info.h>
#include <nrf_modem_at.h>

// PDN library for defining/modifying PDP context information (ie: APN)
#include <modem/pdn.h>          

// Library functions to reboot system
#include <sys/reboot.h>

// Logging 
#include <logging/log.h>
LOG_MODULE_REGISTER(modem);

// Application specific includes
#include "modem.h"
#include "filesystem.h"
#include "led.h"

/*
* Forward declarations for static/private functions in this file 
* This allows us to place global interface functions at top of file
*/

static int modem_rsrp_raw_to_dbm(int rsrp_raw);
static void modem_rsrp_cb(char current_rsrp);
static void modem_lookup_apn(void);
static void pdn_event_handler(uint8_t cid, enum pdn_event event, int reason);

/*
* Define the maximum length of an APN string. This value is from the 3GPP wireless standard and should not have to change.
* 3GPP defines max string to be 63 and we add 1 for the EOS/NULL
*/
#define LTE_APN_LEN		(63+1)

/*
* Define the maximum length of an IMSI string. This value is from the 3GPP wireless standard and should not have to change.
* 3GPP defines max string to be 15 and we add 1 for the EOS/NULL
*/
#define LTE_IMSI_LEN		(15+1)

/* 
* The Nordic PDN libraries use Context ID of 0 for the Default context (pretty typical of most modem vendors) but it's 
* possible that this could change since they don't document this anywhere. 
*/
#define  DEFAULT_CONTEXT_CID 0 


/*
*  Length of MCC/MNC is 6 characters in North America (note: some networks in Europe are 5). This is defined in 3GPP
*/
#define     MCCMNC_LEN      6      

/*
* Define a static buffer for APN string so the caller doesn't have to place on his stack and it's also cached so
* we can fetch and display later if required
*/
static char apn_str[LTE_APN_LEN];   // cache for the APN string
static enum pdn_fam pdn_family;     // cache for PDN family/type


/*
* Define a static buffer for IMSI string so the caller doesn't have to place on his stack. Also we can fetch and display 
* later if required. Note that with IMSI switching SIM cards like Eseye, the IMSI can change during operation if a better
* network is found
*/
static char imsi_str[LTE_IMSI_LEN];

/*
 * Define a structure to create a look-up table of MCC/MNC codes to the corresponding APN and PDN type
 */

struct pdn_context_definition {
    char    mccmnc_characters[MCCMNC_LEN];      // MCC/MNC characters from IMSI, NOTE: not NULL terminated!
    char    apn_str[LTE_APN_LEN];               // APN to be used for this MNC/MCC combination
    enum    pdn_fam pdntype;                    // Type of PDN to be used for this MNC/MCC combination 
};

/**
*   Now create an array of these structures, one structure for each MNC/MCC combination. 
*   Populate (statically initialize) the lookup array table with all the SIM card MCC/MNC combinations that we are currently supporting.
*
*   Easy to add more in the future if you know the MCC/MNC (read from the IMSI at runtime) and the 
*   APN (ask the Mobile Network Operator for it) 
*
*   MCC == Mobile Country Code (Canada is 302, USA is 310)
*   MNC == Mobile Network Code (Rogers uses 720 but has multiple MNCs. This is typical for many Operators)
*
*/
struct pdn_context_definition apn_lookup_table[] = 
{
    // The first entry is Bell, they have multiple MNC codes, I picked 302/610 as an example. 
    // An example APN is defined but it's wrong. Just an example. The PDN type is defined too 
    {{'3', '0', '2','6', '1', '0'}, "bell-example.apn", PDN_FAM_IPV4V6},

    // 2nd entry is Rogers, they use MCC 302 / MNC 720 for IoT profiles
    {{'3', '0', '2','7', '2', '0'}, "m2minternet.apn", PDN_FAM_IPV4V6},

    // 3rd entry is AT&T, MCC is 310 and MNC of 370 (plus many more)
    {{'3', '1', '0', '3', '7', '0'}, "m2m.com.attz", PDN_FAM_IPV4V6},

    // 4th entry is Telus, they use MCC 302 / MNC 221 plus others, example APN and IPV4 pdn
    {{'3', '0', '2', '2', '2', '1'}, "telus-example.apn", PDN_FAM_IPV4},

    // just add more below in the same exact format and the code below will pick it up and scan the entire table
    // There is a special case for SIM cards with IMSI switching like Eseye. See below.  
};

// The PDN library has some callbacks and need to map the info strings.  
static const char * const fam_str[] = {
	[PDN_FAM_IPV4V6] = "IPV4V6",
	[PDN_FAM_IPV6] = "IPV6",
	[PDN_FAM_IPV4] = "IPV4",
};




/*
* Cached Signal RSRP value in dBm
*/
static int modem_rsrp_dbm = 0;

/*
* Storage for callback function to notify of pdp context notification
*/
static void (*pdp_context_notify)(bool);




/******************
*
* Global API interface functions - needed/used by other modules
*
*******************/

/* 
* @brief    Modem function to bring the PDP conext up or down - currently used for debugging Cloud connectivity issues 
*           but could be used for recovery purposes in the future.   
*
* @param    up - flag to signal requested up/down state of the PDP context
*
* @return   nothing
*
* @note     This service has an issue because the Nordic SDK doesn't permit these operations on Context ID == 0 
*/
void modem_pdp_control(bool up)
{

    int err;
    int esm;

    // does the caller want to bring up the PDP context?
    if (up)
    {
        // then activate PDP using the previous determined PDN family type
        err = pdn_activate(DEFAULT_CONTEXT_CID, &esm, &pdn_family);
        LOG_DBG("PDP Context requested: UP, ESM error reason: %d", esm);
    }
    else
    {
        err = pdn_deactivate(DEFAULT_CONTEXT_CID);
        LOG_DBG("PDP Conext requested DOWN");
    }

}

/**
 * @brief Acquires the IMEI of the modem
 * 
 * @param device_imei pointer to char, gets updated if it is a success
 * @param imei_size expected length of the string
 * 
 * @return Does not return any
 */
void modem_get_IMEI(char *device_imei, int imei_size)
{
    LOG_DBG("Acquiring IMEI");

    int ret = modem_info_string_get(MODEM_INFO_IMEI, device_imei, imei_size);
    if (ret < 0)
    {
        LOG_ERR("failed to get IMEI -> err:%d", ret);
    }
    else
    {
        if (strlen(device_imei) < (imei_size - 1))
        {
            LOG_ERR("IMEI Length is less than %d", imei_size);
        }

        LOG_DBG("Device IMEI: %s", device_imei);
    }

}

/**
 * @brief Acquires the ICCID of the modem
 * 
 * @param device_iccid pointer to char, gets updated if it is a success
 * @param imei_size expected length of the string
 * 
 * @return Does not return any
 */
void modem_get_ICCID(char *device_iccid, int iccid_size)
{
    char response[256];

    //CFUN=41 is to activate UICC, to obtain ICCID
    int err = nrf_modem_at_cmd(response, sizeof(response), "AT+CFUN=41"); 
    if(err)
    {
        LOG_DBG("cfun=41 mode not set");
    }

    int ret = modem_info_string_get(MODEM_INFO_ICCID, device_iccid, iccid_size);
    if (ret < 0)
    {
        LOG_ERR("failed to get ICCID -> err:%d", ret);
    }
    else
    {
        if (strlen(device_iccid) < (iccid_size-2))
        {
            LOG_ERR("ICCID Length is less than %d", iccid_size);
        }

        LOG_INF("Device ICCID: %s", device_iccid);
    }
    //turning OFF the sim card
    nrf_modem_at_cmd(response,sizeof(response),"AT+CFUN=40");
}

/**
 * @brief Enables the monitoring of RSRP(non-blocking)
 * 
 * @return Does not return any
 */
void modem_enable_rsrp_monitor()
{
    int err = 0;

    err = modem_info_rsrp_register(modem_rsrp_cb);

    if (err != 0)
    {
        LOG_ERR("Failed to register the callback function for monitoring RSRP -> err:%d", err);
    }
}

/**
 * @brief Gets the RSRP(blocking)
 * 
 * @param 
 * 
 * @return RSRP value in dBm
 */
int modem_get_rsrp_dbm_now(void)
{
    int ret = 0;

    uint16_t rsrp;

    int err = modem_info_short_get(MODEM_INFO_RSRP, &rsrp); 

    if (err < 0)
    {
        LOG_ERR("failed to get RSRP(basic) -> err:%d", ret);
    }
    else
    {
        modem_rsrp_dbm = modem_rsrp_raw_to_dbm(rsrp);
        ret = modem_rsrp_dbm;
    }

    return ret;
}

/**
 * @brief Returns the most updated RSRP value
 * 
 * @return Returns RSRP value, 0 is the initial value
 */
int modem_get_rsrp_dbm(void)
{
    return modem_rsrp_dbm;
}


/**
 * @brief Configures the state of the Modem
 * 
 * @param state Desired state of the Modem
 * 
 * @return Does not return any
 */
int modem_set_state(modem_state_t state)
{
    int err = 0;
    switch(state)
    {
        case MODEM_POWER_ON:    err = lte_lc_normal();
                                break;
        case MODEM_POWER_OFF:   err = lte_lc_power_off();
                                break;
        case MODEM_OFFLINE:     err = lte_lc_offline(); // less flash write than turning it off completely
                                break;
        default:                break;
    }

    return err;
}

/**
 * @brief Sending AT commands to the board and reseting the board
 * 
 */
void modem_factory_reset(bool force)
{
    if (force || false == is_file_exists("modemfactoryreset"))
    {
        // initialising variables
        int err = 0;        // record the response 

        char response[256] = { 0 };  // response buffer

        // command to send AT+CFUN=0 command
        err = nrf_modem_at_cmd(response, sizeof(response), "AT+CFUN=0");
        LOG_INF("CFUN Command successful, 0 means ok:  %i", err);
        
        // command to send AT%XFACTORYRESET=1 command
        err = nrf_modem_at_cmd(response, sizeof(response), "AT%%XFACTORYRESET=1");
        LOG_INF("Reset command successful, 0 means ok:  %i", err);

        // indicate modem reset done
        if(0 > save_to_file("1", sizeof("1"), "modemfactoryreset"))
        {
            led_set_state(RED_LED, true);
            k_msleep(60000);
        }
        else
        {
            led_set_state(GREEN_LED, true);
            k_msleep(500);
        }
        
        // doing system reboot to make sure changes are saved and device is working
        sys_reboot(0);
    }
    
}

/** 
* @brief    modem_init_apn_pdp_context - Initialize APN and PDP context activation handling in Modem
*
*           Configures the APN and PDN type in the modem as well as PDP context activation handler. 
*
*           The  APN values are derived from the SIM provider (Mobile Network Operator).
*           The IMSI field of the SIM card (International Mobile Station Identifier) contains the Mobile Country Code (MCC) and Mobile Network Code (MNC)
*           in it's first 6 bytes (represented as characters). The remaining values identify the subscriber on the mobile network. 
*           Since the APN tends to be an Operator specfic thing, typically the APN can derived from the MCC/MNC. 
*           This is performed via a look-up table that can be extended over time. 
*          
* @param    void
*
* @return   none
*
* @note     Read about PDN concept here: https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/nrf/samples/nrf9160/pdn/README.html
*     
*/
int modem_init_apn_pdp_context(void (*pdp_change_callbackp) (bool flag))
{
    int err;

    // save the callback function pointer from the caller used to notify on pdp context notifications
    pdp_context_notify = pdp_change_callbackp;

	/* Register to the necessary packet domain AT notifications needed for PDN libary */
	err = nrf_modem_at_printf("AT+CNEC=16");
	if (err) {
		LOG_ERR("AT+CNEC=16 failed, err %d\n", err);
		return err;
	}

	err = nrf_modem_at_printf("AT+CGEREP=1");
	if (err) {
		LOG_ERR("AT+CGEREP=1 failed, err %d\n", err);
		return err;
	}

	// initialize the PDN library
	err = pdn_init();
	if (err) {
		LOG_ERR("PDN init failed");
		return err;
	}

	/* Setup a callback for the default PDP context (zero).
	 * Do this before switching to function mode 1 (CFUN=1)
	 * to receive the first activation event.
	 */
	err = pdn_default_callback_set(pdn_event_handler);
	if (err) {
		LOG_ERR("pdn_default_callback_set() failed, err %d\n", err);
		return err;
    }

	/* 
    * Lookup/determine and initialize the APN string buffer and PDN family to be
    * used for this SIM card
	*/
    modem_lookup_apn();
	
	/* Use the Default Context ID and set the APN and PDN Family type
    *  The auto connect feature of the Nordic Link Controller uses Default Conext ID to bring up the connection
    */
    err = pdn_ctx_configure(DEFAULT_CONTEXT_CID, apn_str, pdn_family, NULL);
	if (err) {
		LOG_ERR("pdn_ctx_configure() failed, err %d\n", err);
		return err;
	}

	LOG_DBG("PDP context %d configured: APN %s Family %s", DEFAULT_CONTEXT_CID, apn_str, fam_str[pdn_family]);
    return 0;
 }

 /******************
 *
 * Internal private/secret functions (ie: not global)
 *
 *******************/


/** 
* @brief    Helper function to determine the APN and PDN type based on the IMSI of the SIM card.
*          
* @param    void
*
* @return   none
*
* @note     
*/
static void modem_lookup_apn(void)
{
    struct  pdn_context_definition *pdn;     // create a pointer to a pdn entry in the table (used to loop thru lookup table)
    int     lookup_tbl_sz;                   // number of entries in lookup table
    int     i;                               // for loop counter
    bool    match;                           // flag to exit loop when a match is found       

    // determine how many entries do we have in the table (controls termination of look-up loop)
    lookup_tbl_sz = sizeof(apn_lookup_table)/sizeof(struct pdn_context_definition);

    // get the IMSI from the SIM card so we know what Operator it is
	modem_info_string_get(MODEM_INFO_IMSI, imsi_str, LTE_IMSI_LEN);
	LOG_DBG("SIM card's IMSI: %s", imsi_str);

    match = false;
    // Loop thru look-up table until we find a MNC/MCC match OR we get to the end of the table
    for (i=0; (i < lookup_tbl_sz) && !match; i++)
    {
        pdn = &apn_lookup_table[i];     // point to the next entry in table

        // compare the MCC/MNC of the SIM card IMSI to the entry in the table
        if (0 == memcmp(imsi_str, pdn->mccmnc_characters, MCCMNC_LEN))
            match = true;
    }

    /* 
    *   If 'match' is true, we found an matching entry in the table and the pointer 'pdn' 
    *   will be pointing at the matching entry in the table. 
    *
    *   NOTE: if no match was found, let's assume it's an eseye SIM card. This is IMPORTANT!
    *   as eseye IMSIs can switch/change so we can't count on them. This could be improved 
    *   by perhaps checking ICCID range of the SIM card but that's likely to change overtime 
    *   as well. 
    */

    if (match) {
        strcpy(apn_str, pdn->apn_str);      // copy the APN string from the lookup table
        pdn_family = pdn->pdntype;          // and get/save the PDN type from the table as well
        LOG_DBG("Found MNC/MCC for SIM card. APN setting to %s", apn_str);
    }
    else {
        strcpy(apn_str, "eseye1");          // assume Eseye SIM card, save their APN
        pdn_family = PDN_FAM_IPV4V6;        // Eseye is IPV4V6 and save that too
        LOG_DBG("No matching MNC/MCC found, assuming Eseye");
    }
}

/** 
* @brief    Event handler for PDN module 
*          
* @param    cid - context id
*           event
*           reason
*
* @return   none
*
* @note     
*/

static void pdn_event_handler(uint8_t cid, enum pdn_event event, int reason)
{

	switch (event) {

	case PDN_EVENT_CNEC_ESM:
		LOG_INF("Event: PDP context ESM %d, %d", cid, reason);
		break;

	case PDN_EVENT_ACTIVATED:
		LOG_INF("PDP Context activated, CID = %d", cid);
        pdp_context_notify(true);                                 // signal LTE connection manager that PDP context is up
		break;

	case PDN_EVENT_DEACTIVATED:
        LOG_INF("PDP Context deactivated, CID = %d", cid);
        pdp_context_notify(false);                              // signal lte connection manager that PDP context is down, bump stats
        break;

    case PDN_EVENT_IPV6_UP:
        LOG_INF("PDP IPV6 connectivity up, CID = %d", cid);
        break;

    case PDN_EVENT_IPV6_DOWN:
        LOG_INF("PDP IPV6 connectivity down, CID = %d", cid);
        break;

	default:
		// fatal error handler
		break;
	}
}

/**
 * @brief Converts raw RSRP to dBm
 * 
 * @param rsrp_raw value to be converted
 * 
 * @return Returns the converter RSRP value in dBm
 */
static int modem_rsrp_raw_to_dbm(int rsrp_raw)
{
    int rsrp_dbm = 0;
    rsrp_dbm = rsrp_raw + RSRP_TO_DBM;

    return rsrp_dbm;
}

/**
 * @brief RSRP monitoring callback
 * 
 * @param current_rsrp new/latest value of RSRP
 * 
 * @return Does not return any
 */
static void modem_rsrp_cb(char current_rsrp)
{
    //convert RSRP to dBm
    modem_rsrp_dbm = modem_rsrp_raw_to_dbm(current_rsrp);
}
