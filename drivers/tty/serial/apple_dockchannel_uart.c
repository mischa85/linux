// SPDX-License-Identifier: GPL-2.0
/*
 * Apple DockChannel UART earlycon
 *
 * Apple Silicon exposes a DockChannel UART that carries the debug console
 * out the USB-C SBU pins (debug USB). Add an earlycon for it.
 *
 * The TX FIFO registers sit at device-base + 0x4000; point earlycon there via
 * "reg-offset = <0x4000>" or the offset address (earlycon=dockchannel,mmio32,
 * 0x50882c000). Offsets below are relative to the mapped page.
 */
#include <linux/console.h>
#include <linux/io.h>
#include <linux/serial_core.h>

#define DOCKCHANNEL_TX8		0x04	/* write a byte to transmit */
#define DOCKCHANNEL_TX_FREE	0x14	/* free TX FIFO slots; poll > 0 */

static void dockchannel_uart_putc(struct uart_port *port, unsigned char c)
{
	while (readl(port->membase + DOCKCHANNEL_TX_FREE) == 0)
		cpu_relax();
	writel(c, port->membase + DOCKCHANNEL_TX8);
}

static void dockchannel_uart_early_write(struct console *con, const char *s,
					 unsigned int n)
{
	struct earlycon_device *dev = con->data;

	uart_console_write(&dev->port, s, n, dockchannel_uart_putc);
}

static int __init dockchannel_uart_early_setup(struct earlycon_device *device,
					       const char *opt)
{
	if (!device->port.membase)
		return -ENODEV;

	device->con->write = dockchannel_uart_early_write;
	return 0;
}
OF_EARLYCON_DECLARE(dockchannel, "apple,dockchannel-uart",
		    dockchannel_uart_early_setup);
EARLYCON_DECLARE(dockchannel, dockchannel_uart_early_setup);
