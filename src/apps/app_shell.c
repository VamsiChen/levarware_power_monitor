#include <zephyr.h>
#include <stdlib.h>
 
#include <shell/shell.h>

#include "app_shell.h"
#include "audio.h"
#include "distance.h"
#include "lte_connect_mgr.h"    // need LTE connection mgr to display/clear stats

#include "subsys/modem.h"
 
static int app_shell_audio_print(const struct shell *shell, size_t argc,
                char *argv[])
{
    audio_set_or_toggle_print_dba_state(0, true);
    // /* Received value is a string so do some test to convert and validate it */
    // int32_t desired_awesomeness = -1;
    // if ((strlen(argv[1]) == 1) && (argv[1][0] == '0')) {
    //     desired_awesomeness = 0;
    // }
    // else {
    //     desired_awesomeness = strtol(argv[1], NULL, 10);
    //     if (desired_awesomeness == 0) {
    //         //There was no number at the beginning of the string
    //         desired_awesomeness = -1;
    //     }
    // }
 
    // /* Reject invalid values */
    // if ((desired_awesomeness < 0) || (desired_awesomeness > 65535)) {
    //     shell_fprintf(shell, SHELL_ERROR, "Invalid value: %s; expected [0..65535]\n", argv[1]);
    //     return -1;
    // }
    // /* Otherwise set and report to the user with a shell message */
    // else {
    //     golioth_awesome = (uint16_t)desired_awesomeness;
    //     shell_fprintf(shell, SHELL_NORMAL, "Golioth awesomeness set to: %d\n", desired_awesomeness);
    // }
 
    return 0;
}
 
// static int app_shell_audio_hide(const struct shell *shell, size_t argc,
//                 char *argv[])
// {
//     audio_set_or_toggle_print_dba_state(false);
//     return 0;
// }

/**
 * @brief Handler for Modem reset macro
 * 
 * @return Does not return any
 */
static int thunder_modem_factory_reset(const struct shell *shell, size_t argc,
                char *argv[])
{   
    // function which has AT commands and are sent to device
    modem_factory_reset(true);
    return 0;
}
static int app_shell_distance_print(const struct shell *shell, size_t argc,
                char *argv[])
{   
    // function which has AT commands and are sent to device
    distance_set_verbose_print_state(0, true);
    return 0;
}

/* 
* @brief    Function to display the LTE connection statistics  
*
* @param    shell variable length parameter list
*
* @return   err
*
* @note      
*/
static int app_lte_display(const struct shell *shell, size_t argc,
                char *argv[])
{
    lte_stats_print();
    return 0;
}

/* 
* @brief    Function to clear the LTE connection statistics  
*
* @param    shell variable length parameter list
*
* @return   err
*
* @note      
*/
static int app_lte_clear(const struct shell *shell, size_t argc,
                char *argv[])
{
    lte_stats_clear();
    printk("LTE network statistics cleared\n");
    return 0;
}


void app_shell_init(void)
{
    SHELL_STATIC_SUBCMD_SET_CREATE(
        audio_cmds,
        SHELL_CMD_ARG(print, NULL,
            "toggle printing of live audio dB(A) levels\n"
            "usage:\n"
            "$ audio print\n",
            app_shell_audio_print, 1, 0),
        // SHELL_CMD_ARG(hide, NULL,
        //     "hide the live audio dB(A) levels\n"
        //     "usage:\n"
        //     "$ audio hide",
        //     app_shell_audio_hide, 1, 0),
        SHELL_SUBCMD_SET_END
        );
 
    SHELL_CMD_REGISTER(audio, &audio_cmds, "Print live audio dB(A) output", NULL);

     // Macro for factory reseting the modem if connectivity issue arises
    // thunder_cmds: newly created variable for subcommand
    // thunder_modem)factory_reset: handler to call functions to perform reset
    // thunder: name of the major macro
    SHELL_STATIC_SUBCMD_SET_CREATE(
        thunder_cmds,
        SHELL_CMD_ARG(modem_reset, NULL,
            "Enters the state where AT commands are sent to reset modem.\n"
            "usage:\n"
            "$ thunder modem_reset  \n",
            thunder_modem_factory_reset, 1, 0),
        SHELL_SUBCMD_SET_END
        );

    SHELL_CMD_REGISTER(thunder, &thunder_cmds, "Thunder board commands", NULL);

    SHELL_STATIC_SUBCMD_SET_CREATE(
        distance_cmds,
        SHELL_CMD_ARG(print, NULL,
            "toggle printing of distance measurements\n"
            "usage:\n"
            "$ distance print\n",
            app_shell_distance_print, 1, 0),
        SHELL_SUBCMD_SET_END
        );
    SHELL_CMD_REGISTER(distance, &distance_cmds, "To print live distance output", NULL);


SHELL_STATIC_SUBCMD_SET_CREATE(    
        lte_display_statistics_cmds,
        SHELL_CMD_ARG(display, NULL,
            "displays modem statistics & state\n"
            "usage: lte display\n",
            app_lte_display, 1, 0),

        SHELL_CMD_ARG(clear, NULL,
            "clears LTE connection statistics\n"
            "usage: lte clear\n",
            app_lte_clear, 1, 0),
        
        SHELL_SUBCMD_SET_END
        );
    SHELL_CMD_REGISTER(lte, &lte_display_statistics_cmds, "Shows & clears LTE connection statistics", NULL);


}


