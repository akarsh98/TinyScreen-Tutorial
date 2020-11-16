# TinyCircuits MAX30101 Library

Used for the TinyCircuits **[Pulse Oximetry Sensor](https://tinycircuits.com/collections/wireling-sensors/products/pulse-oximetry-sensor-wireling)**.

*Support this library by buying products from **[TinyCircuits](https://tinycircuits.com/)***


## MAX30101 Class

* **int begin(...)**
* **void setMode(Mode mode)** Sets *mode* to a mode...
* **void setSamplingRateAndAveragingCount(SamplingRate rate, SampleAveragingCount count)** 
* **void setLEDPulseWidth(LEDPulseWidth pw)**
* **void setLEDCurrents( byte redLedCurrent, byte IRLedCurrent )**
* **float readTemperature()**
* **void updateTemperature()**
* **fifo_t readFIFO()**
* **void printRegisters()**
* **void printAlgorithmVals()**
* **bool foundPeak(unsigned long timeOfValue)**

* **dcFilter_t dcRemoval(float x, float prev_w, float alpha)**
* **void lowPassButterworthFilter( float x, butterworthFilter_t \* filterResult )**
* **float meanDiff(float M, meanDiffFilter_t\* filterValues)**

* **bool update()**
* **bool pulseValid()**
* **float BPM()**
* **float cardiogram()**
* **float oxygen()**
* **float temperature()**
* **float temperatureF()**
    
* **float rawCardiogram()**
* **float rawIRVal()**
* **float rawRedVal()**
* **float DCfilteredIRVal()**
* **float DCfilteredRedVal()**

### **private:**
* **bool detectPulse(float sensor_value, unsigned long timeOfValue)**
* **void balanceIntesities( float redLedDC, float IRLedDC )**
* **void writeRegister(byte address, byte val)**
* **uint8_t readRegister(uint8_t address)**
* **void readFrom(byte address, int num, byte _buff[])**


