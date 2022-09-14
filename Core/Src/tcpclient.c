/*
 * tcpclient.c
 *
 *  Created on: 21-Apr-2022
 *      Author: controllerstech
 */


#include "lwip/opt.h"

#include "lwip/api.h"
#include "lwip/sys.h"

#include "tcpclient.h"
#include "string.h"

#include <stdlib.h>

static struct netconn *conn;
static struct netbuf *buf;
static ip_addr_t *addr, dest_addr;
static unsigned short port, dest_port;
char msgc[200];
char smsgc[200];
int indx = 0;
float linear_x=0, angular_z=0;


float *fun(char string[])
{
    static float res[10];
    char *token;
    int count=0;
    token = strtok(string,",");
    float x;
    // res[0] = atof(token);
    while (token)
    {
        x = atof(token);
        res[count] = x;
        // printf("%f ",x);
        count+=1;
        token = strtok(NULL,",");
    }
    return res;
}

// Function to send the data to the server
void tcpsend (char *data);

void set_idx(int ix) {
	indx = ix;
}

float *get_linear_x() {
    return &linear_x;
}

float *get_angular_z() {
	return &angular_z;
}

// tcpsem is the binary semaphore to prevent the access to tcpsend
sys_sem_t tcpsem;

static void tcpinit_thread(void *arg)
{
	err_t err, connect_error;

	/* Create a new connection identifier. */
	conn = netconn_new(NETCONN_TCP);

	if (conn!=NULL)
	{
		/* Bind connection to the port number 7 (port of the Client). */
		err = netconn_bind(conn, IP_ADDR_ANY, 10);

		if (err == ERR_OK)
		{
			/* The desination IP adress of the computer */
			IP_ADDR4(&dest_addr, 10, 42, 0, 1);
			dest_port = 3000;  // server port

			/* Connect to the TCP Server */
			connect_error = netconn_connect(conn, &dest_addr, dest_port);

			// If the connection to the server is established, the following will continue, else delete the connection
			if (connect_error == ERR_OK)
			{
				// Release the semaphore once the connection is successful
				sys_sem_signal(&tcpsem);
				while (1)
				{
					/* wait until the data is sent by the server */
					if (netconn_recv(conn, &buf) == ERR_OK)
					{
						/* Extract the address and port in case they are required */
//						addr = netbuf_fromaddr(buf);  // get the address of the client
//						port = netbuf_fromport(buf);  // get the Port of the client

						/* If there is some data remaining to be sent, the following process will continue */
						do
						{

							strncpy (msgc, buf->p->payload, buf->p->len);   // get the message from the server

							float *linx = fun(msgc);
							linear_x = *(linx);
							angular_z = *(linx+1);

							// Or modify the message received, so that we can send it back to the server

							// semaphore must be taken before accessing the tcpsend function
//							sys_arch_sem_wait(&tcpsem, 5);
//
//							// send the data to the TCP Server
//							tcpsend (smsgc);

							memset (msgc, '\0', 100);  // clear the buffer
						}
						while (netbuf_next(buf) > 0);

						netbuf_delete(buf);
					}
				}
			}

			else
			{
				/* Close connection and discard connection identifier. */
				netconn_close(conn);
				netconn_delete(conn);
			}
		}
		else
		{
			// if the binding wasn't successful, delete the netconn connection
			netconn_delete(conn);
		}
	}
}

void tcpsend (char *data)
{
	// send the data to the connected connection
	netconn_write(conn, data, strlen(data), NETCONN_COPY);
	// relaese the semaphore
	sys_sem_signal(&tcpsem);
}


static void tcpsend_thread (void *arg)
{
	for (;;)
	{
		sprintf (smsgc, "index value = %d\n", indx);
		// semaphore must be taken before accessing the tcpsend function
//		sys_arch_sem_wait(&tcpsem, 5);
		// send the data to the server
		tcpsend(smsgc);
		osDelay(1);
	}
}




void tcpclient_init (void)
{
	sys_sem_new(tcpsem, 0);  // the semaphore would prevent simultaneous access to tcpsend
	sys_thread_new("tcpinit_thread", tcpinit_thread, NULL, DEFAULT_THREAD_STACKSIZE,osPriorityNormal);
	sys_thread_new("tcpsend_thread", tcpsend_thread, NULL, DEFAULT_THREAD_STACKSIZE,osPriorityNormal);
}
