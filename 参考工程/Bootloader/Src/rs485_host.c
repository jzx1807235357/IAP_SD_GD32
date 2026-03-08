#include "rs485_host.h"
#include "rs485_host_cfg.h"
#include "delay.h"

#include <stddef.h>
#include <stdbool.h>

typedef struct
{
    uint32_t usart;
    rcu_periph_enum usart_clk;
    uint32_t baudrate;

    uint32_t tx_port;
    rcu_periph_enum tx_port_clk;
    uint32_t tx_pin;
    uint32_t tx_af;

    uint32_t rx_port;
    rcu_periph_enum rx_port_clk;
    uint32_t rx_pin;
    uint32_t rx_af;

    uint32_t de_port;
    rcu_periph_enum de_port_clk;
    uint32_t de_pin;

    uint16_t tx_delay_us;
    uint16_t rx_delay_us;

    bool initialized;
} rs485_host_context_t;

static rs485_host_context_t s_rs485_host_ctx = {
    RS485_HOST_USART,
    RS485_HOST_USART_CLK,
    RS485_HOST_BAUDRATE,

    RS485_HOST_TX_PORT,
    RS485_HOST_TX_PORT_CLK,
    RS485_HOST_TX_PIN,
    RS485_HOST_TX_AF,

    RS485_HOST_RX_PORT,
    RS485_HOST_RX_PORT_CLK,
    RS485_HOST_RX_PIN,
    RS485_HOST_RX_AF,

    RS485_HOST_DE_PORT,
    RS485_HOST_DE_PORT_CLK,
    RS485_HOST_DE_PIN,

    RS485_HOST_TX_DELAY_US,
    RS485_HOST_RX_DELAY_US,
    false};

static void rs485_host_hw_init(rs485_host_context_t *ctx);
static void rs485_host_set_de_pin(rs485_host_context_t *ctx, bool tx_mode);

void rs485_host_init(void)
{
    if (s_rs485_host_ctx.initialized)
    {
        return;
    }

    rs485_host_hw_init(&s_rs485_host_ctx);
    s_rs485_host_ctx.initialized = true;
    rs485_host_set_direction(RS485_DIRECTION_RECEIVE);
}

void rs485_host_set_direction(rs485_direction_t direction)
{
    if (!s_rs485_host_ctx.initialized)
    {
        return;
    }

    rs485_host_set_de_pin(&s_rs485_host_ctx, (direction == RS485_DIRECTION_TRANSMIT));
}

void rs485_host_send(const uint8_t *buffer, uint32_t length)
{
    if ((buffer == NULL) || (length == 0U) || !s_rs485_host_ctx.initialized)
    {
        return;
    }

    rs485_host_set_de_pin(&s_rs485_host_ctx, true);
    if (s_rs485_host_ctx.tx_delay_us != 0U)
    {
        delay_us(s_rs485_host_ctx.tx_delay_us);
    }

    for (uint32_t i = 0U; i < length; ++i)
    {
        usart_data_transmit(s_rs485_host_ctx.usart, buffer[i]);
        while (RESET == usart_flag_get(s_rs485_host_ctx.usart, USART_FLAG_TBE))
        {
        }
    }

    while (RESET == usart_flag_get(s_rs485_host_ctx.usart, USART_FLAG_TC))
    {
    }

    rs485_host_set_de_pin(&s_rs485_host_ctx, false);
    if (s_rs485_host_ctx.rx_delay_us != 0U)
    {
        delay_us(s_rs485_host_ctx.rx_delay_us);
    }
}

int32_t rs485_host_receive(uint8_t *buffer, uint32_t length, uint32_t timeout_us)
{
    if ((buffer == NULL) || (length == 0U) || !s_rs485_host_ctx.initialized)
    {
        return -1;
    }

    uint32_t remaining = length;
    /* 字节间超时：每个字节最多等待20ms（远大于87μs的单字节传输时间） */
    uint32_t byte_timeout = 20000U;

    while (remaining > 0U)
    {
        uint32_t timeout_ticks = byte_timeout;
        
        while (RESET == usart_flag_get(s_rs485_host_ctx.usart, USART_FLAG_RBNE))
        {
            if (timeout_ticks-- == 0U)
            {
                return (int32_t)(length - remaining);
            }
            delay_us(1U);
        }

        *buffer++ = (uint8_t)usart_data_receive(s_rs485_host_ctx.usart);
        --remaining;
    }

    return (int32_t)length;
}

static void rs485_host_hw_init(rs485_host_context_t *ctx)
{
    if (ctx == NULL)
    {
        return;
    }

    rcu_periph_clock_enable(ctx->tx_port_clk);
    rcu_periph_clock_enable(ctx->rx_port_clk);
    rcu_periph_clock_enable(ctx->de_port_clk);
    rcu_periph_clock_enable(ctx->usart_clk);

    gpio_mode_set(ctx->tx_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, ctx->tx_pin);
    gpio_output_options_set(ctx->tx_port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, ctx->tx_pin);
    gpio_af_set(ctx->tx_port, ctx->tx_af, ctx->tx_pin);

    gpio_mode_set(ctx->rx_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, ctx->rx_pin);
    gpio_af_set(ctx->rx_port, ctx->rx_af, ctx->rx_pin);

    gpio_mode_set(ctx->de_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, ctx->de_pin);
    gpio_output_options_set(ctx->de_port, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, ctx->de_pin);
    gpio_bit_reset(ctx->de_port, ctx->de_pin);

    usart_deinit(ctx->usart);
    usart_baudrate_set(ctx->usart, ctx->baudrate);
    usart_word_length_set(ctx->usart, USART_WL_8BIT);
    usart_stop_bit_set(ctx->usart, USART_STB_1BIT);
    usart_parity_config(ctx->usart, USART_PM_NONE);
    usart_receive_config(ctx->usart, USART_RECEIVE_ENABLE);
    usart_transmit_config(ctx->usart, USART_TRANSMIT_ENABLE);
    usart_enable(ctx->usart);
}

static void rs485_host_set_de_pin(rs485_host_context_t *ctx, bool tx_mode)
{
    if (ctx == NULL)
    {
        return;
    }

    if (tx_mode)
    {
        gpio_bit_set(ctx->de_port, ctx->de_pin);
    }
    else
    {
        gpio_bit_reset(ctx->de_port, ctx->de_pin);
    }
}

