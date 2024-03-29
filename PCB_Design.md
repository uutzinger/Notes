## PCB Design Rule of Thumb:

### Examples Designs: 

- Motor Controllers

    - [ODrive](https://github.com/odriverobotics/ODriveHardware/tree/master/v3/v3.5docs)
    - [VESEC](https://github.com/vedderb/bldc-hardware)

- Sensors:
    - [ECG](https://github.com/sparkfun/AD8232_Heart_Rate_Monitor)
    - [PulseOx](https://github.com/Protocentral/AFE4490_Oximeter/tree/master/Hardware/pc_afe4490_brk_v2)

- If you want to become advanced check out Phil's videos at https://www.phils-lab.net/

### Rules
- Search datasheets of your components for a refrence design. 
- Search for application notes of the components you are using and search for a reference design.
- Search for an evaluation kit of the component you like to use and lookup the circuit that is used in the evalaution kit manual. If available also take a look at the photos showing the layout for potential component placement.
- Seach for open source designs using the component you are using, for example designs from Sparkfun or Adafruit.
- Make sure your schematic is relatively complete.
- Update the design paramters to your preferred settings such as filter frequencies, I2C address, or other external chip configurations.
- Make sure you have all the components you will place in your library. You can use standard elements included in your CAD program but you need to check availability for example on LCSC Electronics, Digikey or Mouser. Use components from LCSC if you plan to have your PCB manufactured by JLPCB.
- Create a plan where on your PCB the digital input and output and where the analog input and output will be, so it will work with the housing you are planning to use. 
- Decide on the size of your PCB. Don't make it too small for the first version of your design.
- Assume you will need to redesing your PCB or that you will need to fix errors. Therefore the components of your first PCB should be accessible and you can shrink the size of the layout later after you verified that it works. 
- Create groups of components for example analog pre processing, digital communication, power regulation, Wifi, Bluetooth and separate those components to avoid cross talk between them. Especially separate noisy power components (e.g. motor controller) from other devices.
- Place your components (resitors, capcacitors, ICs) on the PCB and arrange components in the same direction except if there is an advantage to rotate them.
- Place analog signal components close to their input or output connector. Place analog sensors close to their first amplifier.
- If you have multiple analog signal components place the components close to each other so that you can minimize the path of the signal traces.
- All integrated circuits will likely need one or two decoupling capacitors. Place those close to the Powersupply pin of the IC.
- The feedback resistor and capacitor of a differential amplifier should be placed close to the non inverted and inverted input pin. 
- Make one of your planes/layer a ground plane. Connect components with vias to that plane. You can still place traces on that plane.
- Traces usually go vertical and horizontal and the directions are connected with 45 degree corener to avoid sharp turns. The corner segement is similar in length for all corners.
- Do not make vias if they are not needed. 
- Do not make sharp turns and avoid turns and corners if they are not needed.
- Keep traces short when possible.
- Make 45 degrees traces when you need to fan out the pads of an IC or make a diagonal bus of multiple wires to an other component. 
- You want to shield analog traces with ground plane next to them as well as on the layer underneath them.
- You can use ground planes on both sides of the PCB.
- Keep analog signals separate/away from digital signals (i2c).
- Analog or digital signals should not be close and parallel to a power trace.
- Make power trace (e.g. 3.3V) a little wider than the signal traces to allow for more current.
- Components (e.g. QWICC conenctor) need to be oriented so that the cables going into them are not obstructed by other components
