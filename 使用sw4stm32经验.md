1.错误信息:
  symbol GPIOA could not be resolved
  解决办法: 首先明白这可能并非错误，而是软件设置的问题，解决办法。
    工程->属性->搜索"indexer"->勾选 index unused headers和index all header variants 这两个选项。
    2.错误信息：
    Description	Resource	Path	Location	Type
The float formatting support (-u _printf_float) is not enabled from linker flags	main.c	/banzi_stm32f107_yizhi/src	line 459	Code Analysis Problem
解决办法： 这个也是软件设置的问题，连接器不支持-u_priintf_float函数，在设置里面加上即可， 具体步骤为：右键 ->属性 ->搜索"setting" -> 在MCU GCC Linker中 
加入-u_printf_float即可解决

3 .  错误信息：这个报错信息最坑爹 ，为make:**
Description	Resource	Path	Location	Type
make: *** [stm32f107vct6.elf] Error 1	stm32f107vct6		 	C/C++ Problem
Description	Resource	Path	Location	Type
recipe for target 'stm32f107vct6.elf' failed	makefile	/stm32f107vct6/Debug	line 38	C/C++ Problem
网上查询，说是Makefile文件除了问题，最终查明，是在移植工程的时候，删除了新建工程时候生成的syscalls.c ，而保留了旧的系统的syscalls.c文件导致。