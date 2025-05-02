#include "network.h"
#include "ethernet.h"

#include "tusb.h"

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

//--------------------------------------------------------------------+
// Application API
//--------------------------------------------------------------------+

#if 0
// indicate to network driver that client has finished with the packet provided to network_recv_cb()
void tud_network_recv_renew(void);

// poll network driver for its ability to accept another packet to transmit
bool tud_network_can_xmit(uint16_t size);

// if network_can_xmit() returns true, network_xmit() can be called once
void tud_network_xmit(void *ref, uint16_t arg);
#endif

//--------------------------------------------------------------------+
// Application Callbacks (WEAK is optional)
//--------------------------------------------------------------------+

int _write(int fd, const char *ptr, int len);

// client must provide this: return false if the packet buffer was not accepted
bool tud_network_recv_cb(const uint8_t *src, uint16_t size) {
    if (ethernet_can_transmit(size)) {
        ethernet_send_packet(src, size);
        printf("usb rx %d\n", size);
        return true;
    } else {
        printf("usb rx %d bad\n", size);
        return false;
    }

    /*printf("data sz %d\n", (int)size);
    for (int i = 0; i < size; ++i) {
        printf("%02x", src[i]);
    }
    printf("\n");
    return true;*/
}

// client must provide this: copy from network stack packet pointer to dst
uint16_t tud_network_xmit_cb(uint8_t *dst, void *ref, uint16_t arg) {
    memcpy(dst, ref, arg);

    /*if (memcmp(dst, tud_network_mac_address, 6) == 0) {
        memset(dst, 0, 6); // rewrite destination to be 00:00:00:00:00:00 to make USB driver happy
    }
    memcpy(dst+6, tud_network_mac_address, 6);*/
    // rewrite source address to be from our device :)

    ethernet_free_rx_buffer(ref);
    return arg;
}

//------------- ECM/RNDIS -------------//

// client must provide this: initialize any network state back to the beginning
void tud_network_init_cb(void) {
    printf("netinit\n");
}

// client must provide this: 48-bit MAC address
// TODO removed later since it is not part of tinyusb stack
uint8_t tud_network_mac_address[6] = {
    // :3
    0x42,0x46,0x4f,0x58,0x3a,0x33
    //0x00, 0xe0, 0x4c, 0x68, 0x02, 0xea
};