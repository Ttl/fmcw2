#include <libopencm3/lpc43xx/gpio.h>
#include <libopencm3/lpc43xx/scu.h>
#include "hackrf_core.h"
#include "mcp4022.h"

static uint8_t mcp_val;

#define MCP_PORT GPIO3
#define PIN_CS BIT4
#define PIN_UD BIT5

/* Signals are inverted in hardware */
#define CS_LOW() gpio_set(MCP_PORT, PIN_CS)
#define CS_HIGH() gpio_clear(MCP_PORT, PIN_CS)
#define UD_LOW() gpio_set(MCP_PORT, PIN_UD)
#define UD_HIGH() gpio_clear(MCP_PORT, PIN_UD)

static void wait_us(uint32_t wait);
static void mcp_inc(uint8_t val);
static void mcp_dec(uint8_t val);

/* CS low time: 5us
 * UD toggle frequency: 1 MHz
 * UD low/high time: 500ns */

void mcp_init(void) {
    /* P6_5(GPIO3[4]) = MCP_CS
     * P6_9(GPIO3[5]) = MCP_UD */

    /* Configure the pinmux */
	scu_pinmux(SCU_MCP_CS, SCU_GPIO_NOPULL);
	scu_pinmux(SCU_MCP_UD, SCU_GPIO_NOPULL);

    CS_HIGH();
    UD_HIGH();

    /* Configures the pins as outputs */
	GPIO3_DIR |= (BIT4 | BIT5);

    mcp_val = 32;
}

static void wait_us(uint32_t wait) {
    /* Assumes clock frequency of 204 MHz */
    uint32_t i, j;
    for (i = 0; i < wait; i++) {
        for (j = 0; j < 1020; j++) {
            __asm__("nop");
        }
    }
}

static void mcp_inc(uint8_t val) {
    int i;

    UD_HIGH(); /* Set increment mode */
    CS_LOW(); /* Enable serial */
    wait_us(5); /* Wait for device */
    for (i=0;i<val;i++) {
        UD_LOW();
        wait_us(2);
        UD_HIGH();
        wait_us(2);
    }
    CS_HIGH();
    UD_HIGH();
}

static void mcp_dec(uint8_t val) {
    int i;

    UD_LOW(); /* Set decrement mode */
    CS_LOW(); /* Enable serial */
    wait_us(5); /* Wait for device */
    for (i=0;i<val;i++) {
        UD_HIGH();
        wait_us(1);
        UD_LOW();
        wait_us(1);
    }
    CS_HIGH();
    UD_HIGH();
}

void mcp_set(uint8_t val) {
    if (val > MCP_MAX_VALUE) {
        val = MCP_MAX_VALUE;
    }
    int8_t diff = val-mcp_val;
    if (diff > 0) {
        mcp_inc(diff);
    } else if (diff < 0) {
        mcp_dec(diff);
    }
    mcp_val = val;
}
