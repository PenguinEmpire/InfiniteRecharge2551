#pragma once

#include "frc/I2C.h"
#include "frc/Timer.h"

#include "units/units.h"

class Lidar {
 public:
	Lidar() {
    I2CBus = new frc::I2C(frc::I2C::kOnboard, Lidar::ADDRESS_DEFAULT);
    frc::Wait(1.);
  };

  Lidar(int I2CAddress) {
    I2CBus = new frc::I2C(frc::I2C::kOnboard, I2CAddress);
  }

  Lidar(bool throughNAVX) {
    printf("in lidar bool contructor\n");
    if(throughNAVX) {
      I2CBus = new frc::I2C(frc::I2C::kMXP, Lidar::ADDRESS_DEFAULT);
    } else {
      I2CBus = new frc::I2C(frc::I2C::kOnboard, Lidar::ADDRESS_DEFAULT);
    }
    printf("lidar constructed through bool constructor\n");
    frc::Wait(1.);
  }

  Lidar(frc::I2C::Port port) {
    I2CBus = new frc::I2C(port, Lidar::ADDRESS_DEFAULT);
    printf("lidar constructed through port constructor\n");
    frc::Wait(1.);
  }

  void SetAddress() {
    unsigned char serialNumber[Lidar::READ_2_REGISTERS];
    int newAddress = (int)0x66;

    if (I2CBus->Read(0x96, 2, serialNumber)) {
      printf("Read operation - reading serial number from 0x96 - failed. Line %d\n", __LINE__);
    }

    I2CBus->Write(0x18, serialNumber[0/* maybe 1 */]);
    I2CBus->Write(0x19, serialNumber[1/* maybe 0 */]);

    I2CBus->Write(0x1a, newAddress);
    I2CBus->Write(0x1e, 0x08); // Disable default address
  }

  units::inch_t GetDistance() {
    return units::inch_t(AquireDistance());
  }

private:
	enum Address {ADDRESS_DEFAULT=0x62}; // default I2C bus address for the LIDAR Lite v2
	enum Register {COMMAND=0x00, STATUS=0x01, DISTANCE_1_2=0x8f};
	enum Command {ACQUIRE_DC_CORRECT=0x04};
	enum NumberOfRegistersToRead {READ_1_REGISTER=0x01, READ_2_REGISTERS=0x02};
	enum NumberOfRegistersToWrite {WRITE_1_REGISTER=0x01};
	frc::I2C* I2CBus;

  bool initialized = false;

	bool Busy()	{
		unsigned char Status[Lidar::READ_1_REGISTER];
		unsigned char statusRegister[Lidar::WRITE_1_REGISTER];
		statusRegister[Lidar::WRITE_1_REGISTER-1] = Lidar::STATUS;

		/**********read status**********/
		if ( I2CBus->WriteBulk(statusRegister, Lidar::WRITE_1_REGISTER)) {printf ( "WriteBulk status failed! line %d\n", __LINE__ ); return true;}
		if ( I2CBus->ReadOnly(Lidar::READ_1_REGISTER, Status) ) {printf ( "ReadOnly status failed! line %d\n", __LINE__ ); return true;}
		//printf("Status at line %d %0x, bit0=%0x\n", __LINE__, Status[0], Status[0] & (unsigned char)0x01);
		return (Status[0] & (unsigned char)0x01); // bit 0 is LIDAR Lite v2 busy bit
	};

  unsigned int AquireDistance(/*Timer m_timer*/) {      
    unsigned char distance[Lidar::READ_2_REGISTERS];
    unsigned char distanceRegister_1st[Lidar::WRITE_1_REGISTER];
    distanceRegister_1st[Lidar::WRITE_1_REGISTER - 1] = Lidar::DISTANCE_1_2;

    // printf("Time =   starting Lidar::AquireDistance\n");// , m_timer->Get());

    // {frc::Wait(.0001);} while (Busy());

    // printf("Time =   acquiring distance\n");//  , m_timer->Get());


    /***********acquire distance**********/		//	WriteBulk() also works
    if ( I2CBus->Write(Lidar::COMMAND, Lidar::ACQUIRE_DC_CORRECT) ) {
      printf ( "Write operation failed! line %d\n", __LINE__ ); // initiate distance acquisition with DC stabilization
    } else {
      // printf("write operation succeeded");
    }

    // do{frc::Wait(.0001);} while (Busy());

    // printf("Time =  %f reading distance\n");//, m_timer->Get());

    /**********read distance**********/     // Read() does not work
    if ( I2CBus->WriteBulk(distanceRegister_1st, Lidar::WRITE_1_REGISTER)) {
      // printf ( "WriteBulk distance failed! line %d\n", __LINE__ );
    } else {
      // printf("WriteBulk operation succeeded");
      if ( I2CBus->ReadOnly(Lidar::READ_2_REGISTERS, distance)) {
        printf ("ReadOnly distance failed! line %d\n", __LINE__);
      } else { 
        // printf("readonly operation succeeded");
      }
    }

    unsigned int dist = (unsigned int)(distance[0]<<8) + (unsigned int)(distance[1]);

    // printf("distance %d\n", dist);
    // printf("Time =  %f, Distance= %d (0x%0x)\n", m_timer->Get(), dist, dist);
    return dist;
  };

};