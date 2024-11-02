/*! Copyright(c) 2016 Taiwan TP-LINK Technologies Co.Ltd.
 * File    : sdmz.c
 * Author  : Albert Lin <albert.lin@tp-link.com>
 * Detail  : Implement of Smart DMZ function
 * Version : 1.0
 * Date    : 25 Aug, 2014
 */

/*************************************************************************/
/*                             include                                   */
/*************************************************************************/
#include <linux/module.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/netlink.h>
#include <linux/if_ether.h>
#include <linux/if_vlan.h>
#include <net/neighbour.h>
#include <net/arp.h>
#include <net/sock.h>

#include <linux/version.h>

/*************************************************************************/
/*                             defines                                   */
/*************************************************************************/
#define MAX_MSGSIZE 1024
/*************************************************************************/
/*                             types                                     */
/*************************************************************************/

/*************************************************************************/
/*                             variables                                 */
/*************************************************************************/
static DEFINE_MUTEX(sdmz_nl_mutex);


static	u8 sdmz_mac[6]={0,0,0,0,0,0};
static	u8 sdmz_ip[4]={0,0,0,0};
static	u8 lan_ip[4]={192,168,0,1};
static	u8 netmask[4]={0,0,0,0};

 
int stringlength(char *s);
int err;
struct sock *nl_sk = NULL;
int pid = 0;

/*************************************************************************/
/*                             extern                                    */
/*************************************************************************/


extern u16 (*sdmz_arp_update_hook)(struct neighbour *neigh) __read_mostly;
extern u16 (*sdmz_arp_destroy_hook)(struct neighbour *neigh) __read_mostly;

/*************************************************************************/
/*                             function                                  */
/*************************************************************************/

static int
_clear_sdmz_ip()
{

	memset(sdmz_ip, 0, sizeof(sdmz_ip));	

}




void sendnlmsg(char *message, int pid)
{
    struct sk_buff *skb_1;
    struct nlmsghdr *nlh;
    int len = NLMSG_SPACE(MAX_MSGSIZE);
    int slen = 0;
	
	
    if(!message || !nl_sk || !pid)
    {
        return ;
    }
	
    skb_1 = alloc_skb(len,GFP_KERNEL);
    if(!skb_1)
    {
        printk(KERN_ERR "my_net_link:alloc_skb error\n");
    }
	
	mutex_lock(&sdmz_nl_mutex);

    slen = stringlength(message);
    nlh = nlmsg_put(skb_1,0,0,0,MAX_MSGSIZE,0);
    NETLINK_CB(skb_1).pid = 0;
    NETLINK_CB(skb_1).dst_group = 0;
    message[slen]= '\0';
    memcpy(NLMSG_DATA(nlh),message,slen+1);
    //printk("my_net_link:send message '%s'.\n",(char *)NLMSG_DATA(nlh));
    netlink_unicast(nl_sk,skb_1,pid,MSG_DONTWAIT);

	mutex_unlock(&sdmz_nl_mutex);
	
	}

int stringlength(char *s)
{
    int slen = 0;
    for(; *s; s++)
    {
        slen++;
    }
    return slen;
}

void nl_data_ready(struct sk_buff *__skb)
{
    struct sk_buff *skb;
    struct nlmsghdr *nlh;
    char str[1024];
    int i ;
	u32 mac[6] = {0};
	u32 ip[4] = {0};
	u32 ip_mask[4] = {0};
	/* add by suweilin */
	u32 ip_host[4] = {0};
	/* add end */

	mutex_lock(&sdmz_nl_mutex);
	 
    skb = skb_get (__skb);
    if(skb->len >= NLMSG_SPACE(0))
    {
        nlh = nlmsg_hdr(skb);
        memcpy(str, NLMSG_DATA(nlh), sizeof(str));

		if (0 == strncmp(str, "sdmzc", 5))
		{
        	pid = nlh->nlmsg_pid;
			printk(KERN_WARNING "[ %s ] sdmzc pid is: %d\n", __FUNCTION__, pid);
		}
		else
		{
		 	sscanf(str, "%hhX:%hhX:%hhX:%hhX:%hhX:%hhX %u.%u.%u.%u %u.%u.%u.%u %u.%u.%u.%u",&mac[0], &mac[1], &mac[2], &mac[3],&mac[4],&mac[5]
																,&ip[0], &ip[1], &ip[2], &ip[3]
																,&ip_mask[0], &ip_mask[1],&ip_mask[2],&ip_mask[3]
																,&ip_host[0], &ip_host[1],&ip_host[2],&ip_host[3]);
			
			for (i = 0; i < 6; i++)
				sdmz_mac[i] = mac[i];	

			for (i = 0; i < 4; i++) {
				lan_ip[i] = ip[i];	
				netmask[i] = ip_mask[i];
				/* add by suweilin */
				sdmz_ip[i] = ip_host[i];
				/* add end */
			}
			printk(KERN_WARNING "[ %s ] recv sdmz mac %hhX:%hhX:%hhX:%hhX:%hhX:%hhX\n", __FUNCTION__, sdmz_mac[0], sdmz_mac[1], sdmz_mac[2], sdmz_mac[3], sdmz_mac[4], sdmz_mac[5]);
		}
		_clear_sdmz_ip();	

        kfree_skb(skb);
    }

	mutex_unlock(&sdmz_nl_mutex);
	
 }
 
 
static int
_is_in_subnet(u8 *ip1, u8 *ip2, u8 *mask)
{
	int i;
	
	if(!ip1 || !ip2)
		return 0;

	
	for (i = 0; i < 4; i++) {
		if ((ip1[i] & mask[i]) != (ip2[i] & mask[i]))
				return 0;
	}
		
	return 1;
	
}


static int
_is_the_same_ip(u8 *ip1, u8 *ip2)
{
	if(!ip1 || !ip2)
		return 0;
	
	//if(memcmp(ip1, ip2, sizeof(sdmz_ip)))
	if (ip1[0] == ip2[0] && ip1[1] == ip2[1] && ip1[2] == ip2[2] && ip1[3] == ip2[3] )	
		return 1;
		
	return 0;
	
}

static int
_set_new_sdmz_ip(u8 *ip)
{
	memcpy(sdmz_ip, ip, sizeof(sdmz_ip));
	printk("new sdmz_ip=%02d.%02d.%02d.%02d \n", sdmz_ip[0],sdmz_ip[1],sdmz_ip[2],sdmz_ip[3]);
}




static u16
sdmz_handle_arp_update(struct neighbour *n)
{
	int	sdmz_ip_changed = 0;
	u8	newip[32] = {0};
	int i = 0;
	
	if(!n) 
		return 0;
	
	if(compare_ether_addr(n->ha, sdmz_mac) != 0)
	{
		/* add by suweilin, 如果其他MAC的主机获取了该IP地址，通知上层调整规则*/
		if (_is_the_same_ip(n->primary_key, sdmz_ip) == 1)
		{
			for (i = 0; i < 4; i++)
				sdmz_ip[i] = 0;
			snprintf(newip, 32, "%d.%d.%d.%d", sdmz_ip[0], sdmz_ip[1], sdmz_ip[2], sdmz_ip[3]);

	        sendnlmsg(newip,pid);
		}
	}
	else
	{
		/*
		printk("\n\nsdmz_handle_arp_update\n"); 
		printk("n->mac=%02x:%02x:%02x:%02x:%02x:%02x\n",n->ha[0],n->ha[1],n->ha[2],n->ha[3],n->ha[4],n->ha[5]);
		printk("n->primary_key=%02d.%02d.%02d.%02d\n",n->primary_key[0],n->primary_key[1],n->primary_key[2],n->primary_key[3]);
		*/
		
		if(_is_in_subnet(n->primary_key, lan_ip, netmask) == 0)
			return 0;	
		
		
		if (_is_the_same_ip(n->primary_key, sdmz_ip) == 0) {
			sdmz_ip_changed = 1;
			_set_new_sdmz_ip(n->primary_key);
			snprintf(newip, 32, "%d.%d.%d.%d", n->primary_key[0], n->primary_key[1], n->primary_key[2], n->primary_key[3]);

	        sendnlmsg(newip,pid);
		}	
	}

	return 0;
}


static u16
sdmz_handle_arp_destroy(struct neighbour *n)
{
	
	if(!n) 
		return 0;

	if(compare_ether_addr(n->ha, sdmz_mac) != 0)
		return 0;

	if (_is_the_same_ip(n->primary_key, sdmz_ip) == 0)	
		return 0;	

	/*
	printk("\n\nsdmz_arp_destroy_hook\n");	
	printk("n->mac=%02x:%02x:%02x:%02x:%02x:%02x\n",n->ha[0],n->ha[1],n->ha[2],n->ha[3],n->ha[4],n->ha[5]);
	printk("n->primary_key=%02d.%02d.%02d.%02d\n",n->primary_key[0],n->primary_key[1],n->primary_key[2],n->primary_key[3]);
	*/
	
	_clear_sdmz_ip();
	
     //sendnlmsg("from sdmz_handle_arp_destroy\n",pid);

}


static int __init sdmz_init(void)
{


	nl_sk = netlink_kernel_create(&init_net, NETLINK_SDMZ, 0,
								   nl_data_ready, NULL, THIS_MODULE);
	if (NULL == nl_sk)
	{
		return -1;
	}

	sdmz_arp_update_hook = sdmz_handle_arp_update;
	sdmz_arp_destroy_hook = sdmz_handle_arp_destroy;


	printk("sdmz v1.0.0 loading\n");

	return 0;
}

static void __exit sdmz_exit(void)
{

	sdmz_arp_update_hook = NULL;
	sdmz_arp_destroy_hook = NULL;
	

	mutex_lock(&sdmz_nl_mutex);
	{
	    netlink_kernel_release(nl_sk);
	}
	mutex_unlock(&sdmz_nl_mutex);


	return;
}

module_init( sdmz_init );
module_exit( sdmz_exit );

MODULE_LICENSE("GPL");

MODULE_AUTHOR("Albert Lin <albert.lin@tp-link.com>");
MODULE_DESCRIPTION("The module is used to deal with Smart DMZ functionality.");
