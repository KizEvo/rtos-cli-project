# CLI with freeRTOS
A simple command-line interface for basic tasks with [freeRTOS](https://www.freertos.org/index.html) on [STM32F411E-DISCO](https://www.st.com/en/evaluation-tools/32f411ediscovery.html). Some available commands :
> - turn LED on
> - turn LED off
> - blink LED \<blink_rate_in_ms\>
> - echo \<text\>
<br>

[PuTTy](https://www.putty.org/) is the software used to communicate over serial line. I also tried Hercules (version 3-2-8) but when I'm transmitting a stream of text (> 5 characters) the UART couldn't synchronize properly and may miss a lot of data. But with PuTTy, I can set the **stop bits** of a data frame so it's easier for the MCU to detect the start bit of a frame.

## Preview
- Turn on, off LEDs and set blink rate

![gif1](https://github.com/KizEvo/rtos-cli-project/assets/104358167/71cee17a-6bfb-4e48-b7d3-0b4522c9a805)

- echo text while LEDs are blinking

![gif2](https://github.com/KizEvo/rtos-cli-project/assets/104358167/08f648ad-960a-4e11-b7c8-e6b4ec9a8ccc)
