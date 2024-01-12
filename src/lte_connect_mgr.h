#ifndef LTECONN_H_
#define LTECONN_H_

int lte_connect_init(void);
bool lte_check_pdp_context(void);
void  lte_application_conn_up(bool aws_up);

void lte_stats_print(void);
void lte_stats_clear(void);


#endif /* LTECONN_H_*/