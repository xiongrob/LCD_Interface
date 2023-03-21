# LCD_Interface

A software interface for the LCD_HD44780.

![image](https://user-images.githubusercontent.com/62448777/226702045-08d1b548-e338-40c3-9a45-d9ecd66a16c7.png)

It leverages a hierachical layering of interfaces:
1) Directly to the pins provided
2) An interface that abstracts the hardware interfaces to their corresponding high-level instructions.
3) Then the virtualization of the state of the screen.

The main motivation here is to leverage a bottom-up development that ultimately makes the development and interface easier to program.
The low-level interface allowed me to ensure that I could get the exact behavior desired by directly manpulating the input of the pins
and see whether or not they were behaving as expected. That way I could provide a high-level abstraction, via specific functions,
that hardcodes the exact bit-patterns and not have to deal with possibly incorrecting formatting the bit-patterns.

The functions become instructions that map the right bit-patterns to be ultimately decoded by the low-level hardware (like a decoder for
a pipeline) which had been previously tested to ensure that it was properly sending the right signals, and at the right time.

Thus, through this layered process, I am able to introduce more user-friendly commands that made it much easier to program overall.
