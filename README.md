这是Xilinx ZYNQ7000系列芯片内的逻辑内核AXI DMA(AXI Direct Memory)，是使用Xilinx Vivado 设计工具而设计的软核，在内存和AXI流目标外设之间提供了高带宽的内存直连访问。
AXI DMA(AXI Direct Memory)IP核，具有两种模式，分别为独立分散/聚集模式和直接寄存器模式，具体参考pg021_axi_dma_PG021.pdf文档。
地址偏移 	名称 			描述 
00h 		MM2S_DMACR 		MM2S DMA控制寄存器 
04h 		MM2S_DMASR 		MM2S DMA状态寄存器 
08-14h 		Reserved 		N/A 
18h 		MM2S_SA 		MM2S源地址。 地址的低32位。 
1Ch 		MM2S_SA_MSB 		MM2S源地址。 地址的高32位。 
28h 		MM2S_LENGTH 		MM2S传输长度（字节） 
30h 		S2MM_DMACR 		S2MM DMA控制寄存器 
34h 		S2MM_DMASR 		S2MM DMA状态寄存器 
38-44h 		Reserved 		N/A 
48h 		S2MM_DA 		S2MM目标地址。 低32位地址。 
4Ch 		S2MM_DA_MSB 		S2MM目标地址。 高32位地址。 
58h 		S2MM_LENGTH 		S2MM缓冲区长度（字节）

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
