#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/mm.h>

#define DEVICE_NAME         "axidma"
#define DMA_CAPACITY        1048576//1MB

#define DMA_MM2S_CR         0x0
#define DMA_MM2S_SR         0x4   
#define DMA_MM2S_SADDR      0x18
#define DMA_MM2S_LENGTH     0x28   

#define DMA_S2MM_CR         0x0
#define DMA_S2MM_SR         0x4   
#define DMA_S2MM_SADDR      0x18
#define DMA_S2MM_LENGTH     0x28 

/*********定义IO_CTRL命令********/
#define DMA_RESET_BASE          'b'
#define DMA_READ_RESET          (_IOW(DMA_RESET_BASE,0,int))
#define DMA_WRITE_RESET         (_IOW(DMA_RESET_BASE,1,int))
#define DMA_WRITE_READ_RESET    (_IOW(DMA_RESET_BASE,2,int))

//dma的寄存器访问顺序需固定
#define dma_readreg(addr)           readl(addr)
#define dma_writereg(addr,val)      writel(val,addr)

struct axidma_dev{
    dev_t devid;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    struct platform_device *pdev;
    void *s2mm_virt_addr;
    void *mm2s_virt_addr;
    dma_addr_t s2mm_phy_addr;
    dma_addr_t mm2s_phy_addr;
    void __iomem *s2mm_base;
    void __iomem *mm2s_base;
};

static int axidma_open(struct inode *file, struct file *flip){
    flip->private_data = container_of(file->i_cdev,struct axidma_dev,cdev);
    pr_info("Axi direct dma is prepared\r\n");
    return 0;
}

static int axidma_release(struct inode *file, struct file *flip){
    pr_info("Axi direct dma is exited\r\n");
    return 0;
}

static ssize_t axidma_read(struct file *flip, char __user *buf, size_t cnt, loff_t *offt){
    unsigned int s2mm_status=0;
    unsigned int s2mm_ctrl=0;
    unsigned int virtual_length=0;
    unsigned int i=0;
    struct axidma_dev *axidma_dev = (struct axidma_dev *)flip->private_data;
    
    if(cnt > DMA_CAPACITY)
    {
        pr_err("the number of data is not enough!\n");
        return -EINVAL;
    }
    /*******************给s2mm控制寄存器的中断位、启动位置1、关闭复位***************************/
    s2mm_ctrl = dma_readreg(axidma_dev->s2mm_base+DMA_S2MM_CR);
    s2mm_ctrl |= (1<<0);//开启dma
    s2mm_ctrl &= ~(1<<12);//关闭中断
    s2mm_ctrl &= ~(1<<2);//关闭read dma reset
    dma_writereg(axidma_dev->s2mm_base+DMA_S2MM_CR,s2mm_ctrl);

    s2mm_ctrl = 0;
    /********************************写入目的地址**********************************/
    dma_writereg(axidma_dev->s2mm_base+DMA_S2MM_SADDR,axidma_dev->s2mm_phy_addr);
    /********************************写入传输长度**********************************/
    dma_writereg(axidma_dev->s2mm_base+DMA_S2MM_LENGTH,cnt);

    do {
        i++;
        s2mm_status=dma_readreg(axidma_dev->s2mm_base+DMA_S2MM_SR);
        if(i == 0xffffff)
        {
            virtual_length = dma_readreg(axidma_dev->s2mm_base+DMA_S2MM_LENGTH);
            pr_err("dma error s2mm_sr=0x%08x,s2mm_len=0x%08x\r\n",s2mm_status,virtual_length);
            
            /*******************复位读s2mm的dma*******************/
            s2mm_ctrl = dma_readreg(axidma_dev->s2mm_base+DMA_S2MM_CR);
            s2mm_ctrl |= (1<<2);
            dma_writereg(axidma_dev->s2mm_base+DMA_S2MM_CR,s2mm_ctrl);
            s2mm_ctrl = 0;
            return -EINVAL;
        }
    }while(!(s2mm_status & (1 << 1)));

    virtual_length = dma_readreg(axidma_dev->s2mm_base+DMA_S2MM_LENGTH);
    /********************************关闭dma**********************************/
    s2mm_ctrl = dma_readreg(axidma_dev->s2mm_base+DMA_S2MM_CR); 
    s2mm_ctrl &= ~(1<<0);
    dma_writereg(axidma_dev->s2mm_base+DMA_S2MM_CR,s2mm_ctrl);

    /***************************将数据传递到用户空间****************************/
    if(copy_to_user(buf,axidma_dev->s2mm_virt_addr,cnt))
    {
        pr_err("dma read is error!\n");
        return -EINVAL;
    }

    return virtual_length;
}

static ssize_t axidma_write(struct file *flip, const char __user *buf, size_t cnt, loff_t *offt){
    unsigned int mm2s_status=0;
    unsigned int mm2s_ctrl = 0;
    unsigned int virtual_length = 0;
    unsigned int i = 0;
    struct axidma_dev *axidma_dev = (struct axidma_dev *)flip->private_data;

    if(cnt > DMA_CAPACITY)
    {
        pr_err("the number of data is over dma capacity!\n");
        return -EINVAL;
    }

    if(copy_from_user(axidma_dev->mm2s_virt_addr,buf,cnt))
    {
        pr_err("copy to kernel error\n");
        return -EINVAL;
    }
    /*******************给s2mm控制寄存器的中断位、启动位置1、关闭复位***************************/
    mm2s_ctrl = dma_readreg(axidma_dev->mm2s_base+DMA_MM2S_CR);
    mm2s_ctrl |= (1<<0);//开启dma
    mm2s_ctrl &= ~(1<<12);//关闭中断
    mm2s_ctrl &= ~(1<<2);//关闭read dma reset
    dma_writereg(axidma_dev->mm2s_base+DMA_MM2S_CR,mm2s_ctrl);
    mm2s_ctrl = 0;
    /******************************配置传输地址******************************/
    dma_writereg(axidma_dev->mm2s_base+DMA_MM2S_SADDR,axidma_dev->mm2s_phy_addr);
    /******************************配置传输长度******************************/
    dma_writereg(axidma_dev->mm2s_base+DMA_MM2S_LENGTH,cnt);

    do {
        i++;
        mm2s_status=dma_readreg(axidma_dev->mm2s_base+DMA_MM2S_SR);
        if(i == 0xfffff)
        {
            virtual_length = dma_readreg(axidma_dev->mm2s_base+DMA_MM2S_LENGTH);
            pr_err("dma error mm2s_sr=0x%08x,mm2s_len=0x%08x\n",mm2s_status,virtual_length);
             /*******************复位读s2mm的dma*******************/
            mm2s_ctrl = dma_readreg(axidma_dev->mm2s_base+DMA_MM2S_CR);
            mm2s_ctrl |= (1<<2);
            dma_writereg(axidma_dev->mm2s_base+DMA_MM2S_CR,mm2s_ctrl);
            mm2s_ctrl = 0;

            return -EINVAL;
        }
    }while(!(mm2s_status & (1 << 1)));

    virtual_length = dma_readreg(axidma_dev->mm2s_base+DMA_MM2S_LENGTH);
    /********************************关闭dma**********************************/
    mm2s_ctrl = dma_readreg(axidma_dev->mm2s_base+DMA_MM2S_CR);
    mm2s_ctrl &= ~(1<<0);
    dma_writereg(axidma_dev->mm2s_base+DMA_MM2S_CR,mm2s_ctrl);

    return virtual_length;
}

static long axidma_ioctl(struct file *flip,unsigned int cmd,unsigned long arg){
    unsigned long read_reset_state = 0;
    unsigned long write_reset_state = 0;
    struct axidma_dev *axidma_dev = (struct axidma_dev *)flip->private_data;
    switch (cmd)
    {
        case DMA_READ_RESET:
            /*******************复位读s2mm的dma*******************/
            read_reset_state = dma_readreg(axidma_dev->s2mm_base+DMA_S2MM_CR);
            read_reset_state |= (1<<2);
            dma_writereg(axidma_dev->s2mm_base+DMA_S2MM_CR,read_reset_state);
            pr_info("read_reset_state = 0x%08x\r\n",dma_readreg(axidma_dev->s2mm_base+DMA_S2MM_CR));
            break;
        case DMA_WRITE_RESET:
            /*******************复位写mm2s的dma*******************/
            write_reset_state = dma_readreg(axidma_dev->mm2s_base+DMA_MM2S_CR);
            write_reset_state |= (1<<2);
            dma_writereg(axidma_dev->mm2s_base+DMA_MM2S_CR,write_reset_state);
            pr_info("write_reset_state = 0x%08x\r\n",dma_readreg(axidma_dev->mm2s_base+DMA_MM2S_CR));
            break;
        case DMA_WRITE_READ_RESET:
            /*******************复位读s2mm的dma*******************/
            read_reset_state = dma_readreg(axidma_dev->s2mm_base+DMA_S2MM_CR);
            read_reset_state |= (1<<2);
            dma_writereg(axidma_dev->s2mm_base+DMA_S2MM_CR,read_reset_state);
            
            /*******************复位读mm2s的dma*******************/
            write_reset_state = dma_readreg(axidma_dev->mm2s_base+DMA_MM2S_CR);
            write_reset_state |= (1<<2);
            dma_writereg(axidma_dev->mm2s_base+DMA_MM2S_CR,write_reset_state);

            pr_info("read_reset_state = 0x%08x ; write_reset_state = 0x%08x\r\n",
                        dma_readreg(axidma_dev->s2mm_base+DMA_S2MM_CR),dma_readreg(axidma_dev->mm2s_base+DMA_MM2S_CR));
            break;
        default:break;
    }
    return 0;
}

static int axidma_mmap(struct file *flip,struct vm_area_struct *vma){
    pr_debug("AXI_MMAP open,virt = 0x%lx,phys = 0x%lx,len  = 0x%lx\r\n",vma->vm_start,
                    (vma->vm_pgoff),vma->vm_end-vma->vm_start);
    //由用户空间决定映射的物理页地址
    if(remap_pfn_range(vma,vma->vm_start,vma->vm_pgoff
            ,vma->vm_end-vma->vm_start,vma->vm_page_prot)){
        return -EAGAIN;
    }
    
    return 0;
}

static struct file_operations axidma_ops = {
    .owner = THIS_MODULE,
    .open = axidma_open,
    .write = axidma_write,
    .read = axidma_read,
    .unlocked_ioctl = axidma_ioctl,
    .release = axidma_release,
    .mmap = axidma_mmap,
};

static int axidma_probe(struct platform_device *pdev){
    int ret = 0;
    struct  resource *s2mm_res = NULL,*mm2s_res = NULL;
    struct axidma_dev *axidma_dev = NULL;

    axidma_dev = devm_kzalloc(&pdev->dev,sizeof(struct axidma_dev),GFP_KERNEL);
    if(!axidma_dev){
        ret = PTR_ERR(axidma_dev);
        goto malloc_err;
    }

    s2mm_res = platform_get_resource_byname(pdev,IORESOURCE_MEM,"s2mm");
    axidma_dev->s2mm_base = devm_ioremap_resource(&pdev->dev,s2mm_res);
    if(IS_ERR(axidma_dev->s2mm_base)){
        ret = PTR_ERR(axidma_dev->s2mm_base);
        goto malloc_err;
    }

    mm2s_res = platform_get_resource_byname(pdev,IORESOURCE_MEM,"mm2s");
    axidma_dev->mm2s_base = devm_ioremap_resource(&pdev->dev,mm2s_res);
    if(IS_ERR(axidma_dev->s2mm_base)){
        ret = PTR_ERR(axidma_dev->s2mm_base);
        goto malloc_err;
    }

    axidma_dev->s2mm_virt_addr = dma_alloc_coherent(&pdev->dev,DMA_CAPACITY,&axidma_dev->s2mm_phy_addr,GFP_KERNEL);
    if(!axidma_dev->s2mm_virt_addr){
        ret = PTR_ERR(axidma_dev->s2mm_virt_addr);
        goto malloc_err;
    }

    axidma_dev->mm2s_virt_addr = dma_alloc_coherent(&pdev->dev,DMA_CAPACITY,&axidma_dev->mm2s_phy_addr,GFP_KERNEL);
    if(!axidma_dev->mm2s_virt_addr){
        ret = PTR_ERR(axidma_dev->mm2s_virt_addr);
        goto s2mm_dma_alloc_err;
    }

    
    ret = alloc_chrdev_region(&axidma_dev->devid,0,1,DEVICE_NAME);
    if(ret < 0){
        pr_err("alloc chrdev error\r\n");
        goto mm2s_dma_alloc_err;
    }

    cdev_init(&axidma_dev->cdev,&axidma_ops);
    ret = cdev_add(&axidma_dev->cdev,axidma_dev->devid,1);
    if(ret < 0){
        pr_err("cdev add error\r\n");
        goto alloc_chrdev_err;
    }

    axidma_dev->class = class_create(THIS_MODULE,"axidma_chrdevs");
    if(IS_ERR(axidma_dev->class)){
        ret = PTR_ERR(axidma_dev->class);
        pr_err("class create error\r\n");
        goto cdev_add_error;
    }

    axidma_dev->device = device_create(axidma_dev->class,NULL,axidma_dev->devid,NULL,DEVICE_NAME);
    if(IS_ERR(axidma_dev->device)){
        ret = PTR_ERR(axidma_dev->device);
        pr_err("device create error\r\n");
        goto class_create_err;
    }

    platform_set_drvdata(pdev,axidma_dev);

    pr_info("axi direct dma init success!\r\n");

    return 0;

class_create_err:
    class_destroy(axidma_dev->class);

cdev_add_error:
    cdev_del(&axidma_dev->cdev);

alloc_chrdev_err:
    unregister_chrdev_region(axidma_dev->devid,1);
    
mm2s_dma_alloc_err:
    dma_free_coherent(&pdev->dev,DMA_CAPACITY,axidma_dev->mm2s_virt_addr,axidma_dev->mm2s_phy_addr);

s2mm_dma_alloc_err:
    dma_free_coherent(&pdev->dev,DMA_CAPACITY,axidma_dev->s2mm_virt_addr,axidma_dev->s2mm_phy_addr);

malloc_err:
    mm2s_res = NULL;
    s2mm_res = NULL;
    axidma_dev = NULL;

    return ret;
}

static int axidma_remove(struct platform_device *pdev){
    struct axidma_dev *axidma_dev = platform_get_drvdata(pdev);

    dma_free_coherent(&pdev->dev,DMA_CAPACITY,axidma_dev->s2mm_virt_addr,axidma_dev->s2mm_phy_addr);
    dma_free_coherent(&pdev->dev,DMA_CAPACITY,axidma_dev->mm2s_virt_addr,axidma_dev->mm2s_phy_addr);

    device_destroy(axidma_dev->class,axidma_dev->devid);
    class_destroy(axidma_dev->class);
    cdev_del(&axidma_dev->cdev);
    unregister_chrdev_region(axidma_dev->devid,1);
    
    return 0;
}

static const struct of_device_id axidma_id[] = {
    {.compatible = "yang,axi-direct-dma"},
    {}
};
MODULE_DEVICE_TABLE(of,axidma_id);

static struct platform_driver axidma_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name = "axi-direct-dma driver",
        .of_match_table = axidma_id
    },
    .probe = axidma_probe,
    .remove = axidma_remove
}; 

module_platform_driver(axidma_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yangsen<yangsen2123@163.com>");
MODULE_DESCRIPTION("XILINX ZYNQ AXI DIRECT DMA DRIVER");



