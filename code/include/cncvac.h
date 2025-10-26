
void enableTX();
void disableTX();

void setupInterrupt16();
void setupInterrupt8();
void setupInterrupt4();

unsigned short calculateParity(unsigned short data);
void readDIPSwitchAndConfigurePackets();