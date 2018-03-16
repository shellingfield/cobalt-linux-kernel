/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * MIPS implementation of PCI BIOS services for PCI support.
 */
#include <linux/bios32.h>
#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/types.h>
#include <asm/pci.h>

#ifdef CONFIG_PCI

struct pci_ops *pci_ops;

/*
 * BIOS32 replacement.
 */
unsigned long pcibios_init(unsigned long memory_start,
                           unsigned long memory_end)
{
	return memory_start;
}

/*
 * Following the generic parts of the MIPS BIOS32 code.
 */

int pcibios_present (void)
{
	return pci_ops != NULL;
}

/*
 * Given the vendor and device ids, find the n'th instance of that device
 * in the system.  
 */
int pcibios_find_device (unsigned short vendor, unsigned short device_id,
			 unsigned short index, unsigned char *bus,
			 unsigned char *devfn)
{
	unsigned int curr = 0;
	struct pci_dev *dev;

	for (dev = pci_devices; dev; dev = dev->next) {
		if (dev->vendor == vendor && dev->device == device_id) {
			if (curr == index) {
				*devfn = dev->devfn;
				*bus = dev->bus->number;
				return PCIBIOS_SUCCESSFUL;
			}
			++curr;
		}
	}
	return PCIBIOS_DEVICE_NOT_FOUND;
}

/*
 * Given the class, find the n'th instance of that device
 * in the system.
 */
int pcibios_find_class (unsigned int class_code, unsigned short index,
			unsigned char *bus, unsigned char *devfn)
{
	unsigned int curr = 0;
	struct pci_dev *dev;

	for (dev = pci_devices; dev; dev = dev->next) {
		if ((dev->class & 0xffff00) == class_code) {
			if (curr == index) {
				*devfn = dev->devfn;
				*bus = dev->bus->number;
				return PCIBIOS_SUCCESSFUL;
			}
			++curr;
		}
	}
	return PCIBIOS_DEVICE_NOT_FOUND;
}

const char *pcibios_strerror(int error)
{
	static char buf[32];

	switch (error) {
		case PCIBIOS_SUCCESSFUL:
		case PCIBIOS_BAD_VENDOR_ID:
			return "SUCCESSFUL";

		case PCIBIOS_FUNC_NOT_SUPPORTED:
			return "FUNC_NOT_SUPPORTED";

		case PCIBIOS_DEVICE_NOT_FOUND:
			return "DEVICE_NOT_FOUND";

		case PCIBIOS_BAD_REGISTER_NUMBER:
			return "BAD_REGISTER_NUMBER";

                case PCIBIOS_SET_FAILED:          
			return "SET_FAILED";

                case PCIBIOS_BUFFER_TOO_SMALL:    
			return "BUFFER_TOO_SMALL";

		default:
			sprintf (buf, "PCI ERROR 0x%x", error);
			return buf;
	}
}

/*
 * The functions below are machine specific and must be reimplented for
 * each PCI chipset configuration.  We just run the hook to the machine
 * specific implementation.
 */
unsigned long pcibios_fixup (unsigned long memory_start,
                             unsigned long memory_end)
{
	return pci_ops->pcibios_fixup(memory_start, memory_end);
}

int pcibios_read_config_byte (unsigned char bus, unsigned char dev_fn,
                              unsigned char where, unsigned char *val)
{
	return pci_ops->pcibios_read_config_byte(bus, dev_fn, where, val);
}

int pcibios_read_config_word (unsigned char bus, unsigned char dev_fn,
                              unsigned char where, unsigned short *val)
{
	return pci_ops->pcibios_read_config_word(bus, dev_fn, where, val);
}

int pcibios_read_config_dword (unsigned char bus, unsigned char dev_fn,
                               unsigned char where, unsigned int *val)
{
	return pci_ops->pcibios_read_config_dword(bus, dev_fn, where, val);
}

int pcibios_write_config_byte (unsigned char bus, unsigned char dev_fn,
                               unsigned char where, unsigned char val)
{
	return pci_ops->pcibios_write_config_byte(bus, dev_fn, where, val);
}

int pcibios_write_config_word (unsigned char bus, unsigned char dev_fn,
                               unsigned char where, unsigned short val)
{
	return pci_ops->pcibios_write_config_word(bus, dev_fn, where, val);
}

int pcibios_write_config_dword (unsigned char bus, unsigned char dev_fn,
                                unsigned char where, unsigned int val)
{
	return pci_ops->pcibios_write_config_dword(bus, dev_fn, where, val);
}

#endif /* defined(CONFIG_PCI) */
