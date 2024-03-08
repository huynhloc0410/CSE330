# CSE330

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdbool.h>
#include <sys/mman.h>
#include <unistd.h>
#include <linux/ioctl.h>
#include <sys/ioctl.h>
#include "../common.h"

char* devname = "/dev/memalloc";
int   devfd = -1;

struct ioctl_data {
	int number;
};

bool open_device_driver(void) {
    devfd = open(devname, O_RDWR);
    if (devfd < 0) {
        printf("Error: could not open device (%s)\n", devname);
        return false;
    }
    return true;
}

int main(void)
{
    printf("memallocdev\n");
    if (!open_device_driver()) {
        return -1;
    }
    struct ioctl_data* test = (struct ioctl_data*) malloc(sizeof(struct ioctl_data));
    test->number = 0xdeadbeef;
    if (!ioctl(devfd, EXAMPLE_IOCTL, test) < 0) {
        return false;
    }    
    close(devfd);
    return 0;
}





/* General headers */
#include <linux/kthread.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kallsyms.h>
#include <linux/skbuff.h>
#include <linux/freezer.h>
#include <linux/mutex.h>
#include <linux/cdev.h>
#include <linux/mm.h>
#include <linux/sched/mm.h>
#include <linux/highmem.h>
#include <asm/pgtable.h>
#include <asm/tlbflush.h>
#include <linux/vmalloc.h>
#include <asm/pgalloc.h>

/* File IO-related headers */
#include <linux/fs.h>
#include <linux/bio.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/blkdev.h>

/* Sleep and timer headers */
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/sched/types.h>
#include <linux/pci.h>

#include "../common.h"

/* Simple licensing stuff */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jair Kraul");
MODULE_DESCRIPTION("Project 2, CSE 330 Spring 2024");
MODULE_VERSION("0.01");

/* Calls which start and stop the ioctl teardown */
bool memalloc_ioctl_init(void);
void memalloc_ioctl_teardown(void);

/* Project 2 Solution Variable/Struct Declarations */
#define MAX_PAGES           4096
#define MAX_ALLOCATIONS     100

/* Page table allocation helper functions defined in kmod_helper.c */
pud_t*  memalloc_pud_alloc(p4d_t* p4d, unsigned long vaddr);
pmd_t*  memalloc_pmd_alloc(pud_t* pud, unsigned long vaddr);
void    memalloc_pte_alloc(pmd_t* pmd, unsigned long vaddr);

#if defined(CONFIG_X86_64)
    #define PAGE_PERMS_RW		PAGE_SHARED
    #define PAGE_PERMS_R		PAGE_READONLY
#else
    #define PAGE_PERMS_RW		__pgprot(_PAGE_DEFAULT | PTE_USER | PTE_NG | PTE_PXN | PTE_UXN | PTE_WRITE)
    #define PAGE_PERMS_R		__pgprot(_PAGE_DEFAULT | PTE_USER | PTE_NG | PTE_PXN | PTE_UXN | PTE_RDONLY)
#endif

struct alloc_info           alloc_req;
struct free_info            free_req;

/* Init and Exit functions */
static int __init memalloc_module_init(void) {
    printk("Hello from the memalloc module!\n");
    memalloc_ioctl_init();
    memalloc_chkmem_mapped();
    return 0;
}

static void __exit memalloc_module_exit(void) {
    /* Teardown IOCTL */
    memalloc_ioctl_teardown();
    printk("Goodbye from the memalloc module!\n");
}

/* IOCTL handler for vmod. */
static long memalloc_ioctl(struct file *f, unsigned int cmd, unsigned long arg) {
    switch (cmd)
    {
    case ALLOCATE:    	
        /* allocate a set of pages */
        printk("IOCTL: alloc(%lx, %d, %d)\n", alloc_req.vaddr, alloc_req.num_pages, alloc_req.write);
        break;
    case FREE:    	
        /* free allocated pages */
    	printk("IOCTL: free(%lx)\n", free_req.vaddr);
    	break;    	
    default:
        printk("Error: incorrect IOCTL command.\n");
        return -1;
    }
    return 0;
}

/* Required file ops. */
static struct file_operations fops = 
{
    .owner          = THIS_MODULE,
    .unlocked_ioctl = memalloc_ioctl,
};

/* Structures used to create a virtual device */
static dev_t                dev = 0;
static struct class*        memallocdev_class;
static struct cdev          memallocdev_cdev;

/* Initialize the module for IOCTL commands */
bool memalloc_ioctl_init(void) {
    if (alloc_chrdev_region(&dev, 0, 1, "memalloc") < 0) {
        printk("error: couldn't allocate chardev region.\n");
        return -1;
    }
    printk("[*] Allocated chardev.\n");

    /* Initialize the chardev with my fops. */
    cdev_init(&memallocdev_cdev, &fops);

    if (cdev_add(&memallocdev_cdev, dev, 1) < 0) {
        printk("[x] Couldn't add virtdev cdev.\n");
        goto cdevfailed;
    }
    printk("[*] Allocated cdev.\n");

    if ((memallocdev_class = class_create("memallocdev_class")) == NULL) {
        printk("[X] couldn't create class.\n");
        goto cdevfailed;
    }
    printk("[*] Allocated class.\n");

    if ((device_create(memallocdev_class, NULL, dev, NULL, "memalloc")) == NULL) {
        printk("[X] couldn't create device.\n");
        goto classfailed;
    }
    printk("[*] Device opened successfully.\n");

    return 0;

classfailed:
    class_destroy(memallocdev_class);
cdevfailed:
    unregister_chrdev_region(dev, 1);

    return -1;
}

void memalloc_ioctl_teardown(void) {
    /* Destroy the classes too (IOCTL-specific). */
     if (memallocdev_class) {
        device_destroy(memallocdev_class, dev);
        class_destroy(memallocdev_class);
    }
    cdev_del(&memallocdev_cdev);
    unregister_chrdev_region(dev,1);
    printk("[*] Virtual device removed.\n");
}

void memalloc_chkmem_mapped(void){
     
     unsigned long vaddr = free_req.vaddr ;
    pgd_t *pgd;
    p4d_t *p4d;
    pud_t *pud;
    pmd_t *pmd;
    pte_t *pte;
    
    /* Return pointer of the PGD. mm is the mm_struct of the process, address is the logical 
       address in the virtual memory space*/
    pgd = pgd_offset(current->mm, free_req.vaddr);
    if (pgd_none(*pgd)) {
        printk("Error: pgd should always be mapped (something is really wrong!).\n");
        goto out;
    }
    printk("PGD is allocated. \n");

    /* Return pointer to the P4D. pgd is the pointer of PGD, address is the logical address in 
       the virtual memory space.*/
    p4d = p4d_offset(pgd, free_req.vaddr);
    if (p4d_none(*p4d) || p4d_bad(*p4d)) {
        printk("No P4D allocated; page must be unmapped.");
        goto out;
    }
    printk("P4D is allocated. \n");

    /* Return pointer to the PUD. p4d is the pointer of P4D, address is the logical address in 
       the virtual memory space.*/
    pud = pud_offset(p4d, free_req.vaddr);
    if (pud_none(*pud)) {
        printk("No PUD allocated; page must be unmapped.");
        goto out;
    }
    printk("PUD is allocated. \n");


    /* Return pointer to the PMD. pud is the pointer of PUD, address is the logical address in 
       the virtual memory space.*/
    pmd = pmd_offset(pud, free_req.vaddr);
    if (pmd_none(*pmd)) {
        printk("No PMD allocated; page must be unmapped.");
        goto out;
    }
    printk("PMD is allocated. \n");

    /* Return pointer to the PTE. pmd is the pointer of PMD, address is the logical address in 
       the virtual memory space*/
    pte = pte_offset_kernel(pmd, free_req.vaddr);
    if (pte_none(*pte)) {
        printk("No PTE allocated; page must be unmapped.");
        goto out;
    }
    printk("PTE is allocated. \n");

    if (pte_present(*pte)) {
    	printk("mapped page.");
        goto out;
    }
    printk("unmapped page. \n");    // Get a free page for the process
    
    gfp_t gfp = GFP_KERNEL_ACCOUNT;
    void *virt_addr = (void*) get_zeroed_page(gfp);
    if (!virt_addr) {
        printk(KERN_ERR "Failed to allocate memory using vmalloc\n");
        return -ENOMEM;
    }

    // Get the physical address of the page
    unsigned long paddr = __pa(virt_addr);

out:
    return 0;
    
}

module_init(memalloc_module_init);
module_exit(memalloc_module_exit);
















obj-m += memalloc.o
memalloc-y += memalloc-main.o memalloc-helper.o

all:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
user:
	gcc memalloc-main.c -o memallocdev-user

install:
	sudo insmod memalloc.ko

remove: 
	sudo rmmod memalloc
