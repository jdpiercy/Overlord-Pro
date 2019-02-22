
#ifndef fitting_bed_h
#define fitting_bed_h


#ifdef SoftwareAutoLevel

  #define NodeNum 6
  #define ADDING_Z_FOR_POSITIVE 50

extern float plainFactorA, plainFactorB, plainFactorC;
extern float plainFactorABackUp, plainFactorBBackUp, plainFactorCBackUp;
extern float fittingBedArray[NodeNum][3];


extern float plainFactorAAC;
extern float plainFactorBBC;
extern float plainFactorCAC;
extern float plainFactorCBC;

extern float fittingBedOffset[NodeNum];
extern float fittingBedSensorOffset[NodeNum];
extern void fittingBedOffsetInit();
extern void fittingBedOffSensorsetInit();


// Ax+By+Cz+1=0
extern void fittingBedUpdateK();
extern void fittingBedResetK();

extern void fittingBed();
extern void fittingBedResetBackUp();

extern void fittingBedReset();
extern bool fittingBedRaw(uint8_t nodeNumber);

extern void fittingBedArrayInit();

extern bool isSoftwareAutoLevel;

#endif

#endif
