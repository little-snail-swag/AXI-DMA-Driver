这是Xilinx ZYNQ7000系列芯片内的逻辑内核AXI DMA(AXI Direct Memory)，是使用Xilinx Vivado 设计工具而设计的软核，在内存和AXI流目标外设之间提供了高带宽的内存直连访问。
AXI DMA(AXI Direct Memory)IP核，具有两种模式，分别为独立分散/聚集模式和直接寄存器模式，具体参考pg021_axi_dma_PG021.pdf文档。

该驱动采用字符设备框架来构建，axidma_write/axidma_read接口函数分别对应直接寄存器模式的mm2s和s2mm，axidma_ioctl为提供复位读/写的AXI DMA接口函数，axidma_mmap为将物
理地址映射到用户空间进行访问的接口函数。

设备树节点：
axi_direct_dma {
			compatible = "yang,axi-direct-dma";
			reg = <0x40400000 0x30>,<0x40400030 0x30>;
			reg-names = "s2mm","mm2s";
			status = "okay";
};
reg属性必须针对于对应ZYNQ系列芯片映射出AXI DMA的设备地址进行修改！
