/*=============================================================
    A simple example of char device drivers
 ============================================================*/
#include <linux/module.h>
#include <linux/types.h> /* size_t */
#include <linux/fs.h>  /* everything */
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <asm/io.h>
//#include <asm/system.h> /* cli(), *_flags */
#include <asm/uaccess.h>/* copy_from/to_user() */
#include <linux/version.h>

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>

#include <linux/slab.h> /* kmalloc()*/





#include <linux/module.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>

#include <linux/init.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>

#include <asm/uaccess.h>


#define SIMPLECHAR_SIZE  0x1000  /* size of virtual device memory 4KB */
#define MEM_CLEAR 0x1  /*  ioctl operation code */
#define SIMPLECHAR_MAJOR 233    /*  static device major */

//test axi address
#define XPAR_MEMORY_MAPPER_0_S00_AXI_BASEADDR 0x43c00000
#define XPAR_MEMORY_MAPPER_0_S00_AXI_HIGHADDR 0x43C0FFFF
#define mWriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))

#define mReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))


typedef struct mem_dev {
    uint32_t base_addr;
    uint32_t high_addr;
    uint32_t dev_id;
    uint32_t n_regs;
    uint32_t reg_offset;
} mem_dev;
mem_dev* pregs;
void *mappedCmd;

/// Read to and from addresses
u32 Xil_In32(u32 Addr)
{
  return *(volatile u32 *) Addr;
}

void Xil_Out32(u32 OutAddress, u32 Value)
{
  *(volatile u32 *) OutAddress = Value;
}


void InitOLED_Driver(void)
{
    pregs = (struct mem_dev*)kmalloc(sizeof(struct mem_dev), GFP_KERNEL);
    pregs->base_addr = XPAR_MEMORY_MAPPER_0_S00_AXI_BASEADDR;
    pregs->high_addr = XPAR_MEMORY_MAPPER_0_S00_AXI_HIGHADDR;
    pregs->dev_id = 0;
    pregs->n_regs = 20;
    pregs->reg_offset = 4;
    mappedCmd = ioremap(pregs->base_addr,pregs->high_addr-pregs->base_addr+sizeof(pregs->base_addr));
    printk("ioremapping completed...\n");

    //OLED initialization
    mWriteReg((u32)mappedCmd,64,1);
}
int WriteReg_OLED_Driver(const char* pStr, int len)
{
    if(len == 0)
    {
        return 0;
    }
    else if(len > 64)
        len = 64;

    int len4 = len - len%4;

    u32 val = 0;
    int i=0;
    for(i=0;i<len4;i++)
    {
        val |= pStr[i]<<((i%4)<<3);// x<<0-8-16-24

        if((i+1)%4 == 0 && i>0)
        {
            mWriteReg((u32)mappedCmd,i-3,val);
            val = 0;
        }
    }

    if(len%4)
    {
        char len4bottom[4] = {0};
        int lenLeft = len -len4;

        int j=0;
        for(j=0;j<lenLeft;j++)
            len4bottom[j] = pStr[len4+j];

        u32 val = 0;
        val |= len4bottom[0];
        val |= len4bottom[1]<<8;
        val |= len4bottom[2]<<16;
        val |= len4bottom[3]<<24;
        mWriteReg((u32)mappedCmd,len4,val);
    }

    return len4/4+1;
}
void readReg_OLED_Driver(char* pStrOut,int RegCnt)
{
    uint32_t ret[8];
    int i=0;
    for(i=0;i<RegCnt;i++)
    {
        ret[i] = mReadReg((u32)mappedCmd, i*4);
        pStrOut[i*4+0] = ret[i]&0xff;
        pStrOut[i*4+1] = (ret[i]>>8)&0xff;
        pStrOut[i*4+2] = (ret[i]>>16)&0xff;
        pStrOut[i*4+3] = (ret[i]>>24)&0xff;
    }
}

static int axi_cdev_major = SIMPLECHAR_MAJOR;

/*simplechar : virtual char device struct */
struct axi_cdev
{
  struct cdev cdev; /*cdev */
  unsigned char mem[SIMPLECHAR_SIZE]; /*virtual circle memory */
  struct semaphore sem;//for concurrency control
};

struct axi_cdev *paxi_cdev;

int axi_cdev_open(struct inode *inode, struct file *filp)
{
  /* assign the virtual device struct to file private data */
    filp->private_data = paxi_cdev;
    return 0;
}
/* release funtion */
int axi_cdev_release(struct inode *inode, struct file *filp)
{
  return 0;
}

/* ioct operation function */
//static int simplechar_ioctl(struct inode *inodep, struct file *filp, unsigned int cmd, unsigned long arg)
static int axi_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
//  struct axi_cdev *dev = filp->private_data;

//  switch (cmd)
//  {
//    case MEM_CLEAR:
//      if(down_interruptible(&dev->sem)){//check semaphore
//    return - ERESTARTSYS;
//      }

//      memset(dev->mem, 0, SIMPLECHAR_SIZE);
//      up(&dev->sem);//release semaphore

//      printk(KERN_INFO "globalmem is set to zero\n");
//      break;

//    default:
//      return  - EINVAL;
//  }`
  return 0;
}

/* read device */
static ssize_t axi_cdev_read(struct file *filp, char __user *buf, size_t size, loff_t *ppos)
{
  //printk(KERN_ALERT "READ %d \n", size);
  unsigned long p =  *ppos;
  unsigned int count = size;
  int ret = 0;
  struct axi_cdev *dev = filp->private_data;


  if (p >= SIMPLECHAR_SIZE)
    return count ?  - ENXIO: 0;
  if (count > SIMPLECHAR_SIZE - p)
    count = SIMPLECHAR_SIZE - p;

  if(down_interruptible(&dev->sem)){
    printk(KERN_INFO "reading confilict! \n");
    return - ERESTARTSYS;
  }

  if (copy_to_user(buf, (void*)(dev->mem + p), count))
  {
    ret =  - EFAULT;
  }
  else
  {
    *ppos += count;
    ret = count;

    printk(KERN_INFO "read %u bytes(s) from %lu\n", count, p);
  }

  up(&dev->sem);

  return ret;
}

/* write device */
static ssize_t axi_cdev_write(struct file *filp, const char __user *buf,
  size_t size, loff_t *ppos)
{
  unsigned long p =  *ppos;
  unsigned int count = size;
  int ret = 0;
  struct axi_cdev *dev = filp->private_data;


  if (p >= SIMPLECHAR_SIZE)
    return count ?  - ENXIO: 0;
  if (count > SIMPLECHAR_SIZE - p)
    count = SIMPLECHAR_SIZE - p;

  if(down_interruptible(&dev->sem)){
    printk(KERN_INFO "writing confilict! \n");
    return - ERESTARTSYS;
  }



  if (copy_from_user(dev->mem + p, buf, count))
    ret =  - EFAULT;
  else
  {
    *ppos += count;
    ret = count;
    WriteReg_OLED_Driver(dev->mem,count);
    printk(KERN_INFO "written %u bytes(s) from %lu\n", count, p);
  }

  up(&dev->sem);
  return ret;
}


static loff_t axi_cdev_llseek(struct file *filp, loff_t offset, int orig)
{
  loff_t ret = 0;
  switch (orig)
  {
    case 0:  //starting location
      if (offset < 0)
      {
        ret =  - EINVAL;
        break;
      }
      if ((unsigned int)offset > SIMPLECHAR_SIZE)
      {
        ret =  - EINVAL;
        break;
      }
      filp->f_pos = (unsigned int)offset;
      ret = filp->f_pos;
      break;
    case 1:   /* current location */
      if ((filp->f_pos + offset) > SIMPLECHAR_SIZE)
      {
        ret =  - EINVAL;
        break;
      }
      if ((filp->f_pos + offset) < 0)
      {
        ret =  - EINVAL;
        break;
      }
      filp->f_pos += offset;
      ret = filp->f_pos;
      break;
    default:
      ret =  - EINVAL;
      break;
  }
  return ret;
}

/* file_operations*/
static const struct file_operations axi_cdev_fops =
{
  .owner = THIS_MODULE,
  .llseek = axi_cdev_llseek,
  .read = axi_cdev_read,
  .write = axi_cdev_write,
  //.ioctl = axi_cdev_ioctl,//for kernel versions before 2.6, use this
  .unlocked_ioctl=axi_cdev_ioctl,
  .open = axi_cdev_open,
  .release = axi_cdev_release,
};

/* initialize and register cdev */
static void axi_cdev_setup_cdev(struct axi_cdev *dev, int index)
{
  int err, devno = MKDEV(axi_cdev_major, index);

  cdev_init(&dev->cdev, &axi_cdev_fops);
  InitOLED_Driver();
  dev->cdev.owner = THIS_MODULE;
  //dev->cdev.ops = &axi_cdev_fops;
  err = cdev_add(&dev->cdev, devno, 1);
  if (err)
    printk(KERN_NOTICE "Error %d adding LED%d", err, index);
}

/*module initialization function*/
int axi_cdev_init(void)
{
  int result;
  dev_t devno = MKDEV(axi_cdev_major, 0);

  /* staticly allocate major number*/
  if (axi_cdev_major)
  {
    result = register_chrdev_region(devno, 1, "axichar");
    printk("axichar:1");
  }
  else  /*dynamically allocate major number*/
  {
    printk("axichar:!1");
    result = alloc_chrdev_region(&devno, 0, 1, "axichar");
    axi_cdev_major = MAJOR(devno);
  }
  if (result < 0)
  {
    printk("result<0");
    return result;
  }
  
  /* dynamically allocate memory for device struct*/
  paxi_cdev = (struct axi_cdev*)kmalloc(sizeof(struct axi_cdev), GFP_KERNEL);
  if (!paxi_cdev)    /* if fail*/
  {
    result =  - ENOMEM;
    goto fail_malloc;
  }
  memset(paxi_cdev, 0, sizeof(struct axi_cdev));

  axi_cdev_setup_cdev(paxi_cdev, 0);//call the setup method
  //init_MUTEX(&paxi_cdev->sem);//not working on current kernel version, use sema_init() instead
  sema_init(&paxi_cdev->sem,1);

  printk(KERN_INFO "Init simplechar success!\n");
  return 0;

  fail_malloc: unregister_chrdev_region(devno, 1);
  printk("fail_malloc");
  return result;
}

/* module exit function */
void axi_cdev_exit(void)
{
  cdev_del(&paxi_cdev->cdev);   /*unregister cdev*/
  kfree(paxi_cdev);     /*release memory for device struct*/
  unregister_chrdev_region(MKDEV(axi_cdev_major, 0), 1); /*release major number*/
  printk(KERN_INFO "Bye axichar!\n");
}

MODULE_AUTHOR("codehiker");
MODULE_LICENSE("Dual BSD/GPL");

module_param(axi_cdev_major, int, S_IRUGO);

module_init(axi_cdev_init);
module_exit(axi_cdev_exit);



