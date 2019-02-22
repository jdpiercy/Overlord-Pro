
#ifndef SDUPS_h
#define SDUPS_h



#include "ConfigurationStore.h"
#define SDUPSValidateSize 64
#define SDUPSPositionSize 64

bool SDUPSRetrieveClass();
void SDUPSScanPosition();
uint32_t SDUPSRetrievePosition(int8_t theIndex);
void SDUPSStorePosition(uint32_t theFilePosition);
void SDUPSOverlapPosition(uint32_t theFilePosition);
void SDUPSRemovePosition(int8_t theIndex);

boolean SDUPSIsWorking();
boolean SDUPSIsStarting();

void SDUPSStart();

void SDUPSDone();


#endif
