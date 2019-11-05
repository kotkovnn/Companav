/* ������ �������������� � ��������������� ������������� �������� "��������-2"
 * ����������:
 * 	- ����� ������ �� ���������� � ������������ � �������� ����������,
 * 	- ������ �� � ��������������� ���������,
 * 	- �������������� ���� ������ (������-������� -> �������� ����������) - ����� ������,
 * 	- ���������� ��������������� ����� � g_Robot,
 * 	- ���������� ����������� (����� �������� � ��.).
 * ������:
 * 	- ����� CCompanavHandler (�������� ���� ������ � �������),
 * 	- �����, �������������� ����������� ��������� ������,
 * 	- ��� ���������� �������� ��� �������� ����������� ������.
*/

// TODO: ���������� ������-������ 

#include "CLog.h"
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <iosfwd>
#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include "config.h"
#include <math.h>
#include <vector>

#include <eigen/Eigen/Dense>
//#include "CNavigator.h"
#define pi 3.14159265358979
using namespace std;


#ifndef CCOMPANAVHANDLER_H_
#define CCOMPANAVHANDLER_H_

#define LAT_LONG_LOG_HTM "/root/navi_data.htm"
#define LAT_LONG_LOG_TXT "/root/navi_data.txt"
extern CLog g_Log;


// ���������� ������������ ����� ��������� � ������
// + �������� ���������� ������, ����������� �� ���������� ��������������� � ���������
// - ����� ��������� �������� ��������� � ���������
// ! � ���� ������ �������� ���������� �� ������ �������������� ������ ������������
//    ��� ����� � ��� �� �������� ������ 
// ! ���������, �������� �� ��� � ������ ������� �����������
// ������������ - "������ �����������" ������� �������� � ������������ � ��� ������
// ������, ����������� ��� gcc, ������: __attribute__((aligned (n))); 
#pragma pack (1) 
	
	// ���� ������� ������, ����������� �� ����������:
	// �����  �0  ������������  �����������  �������������  ����������  �  �������� 50  ��. 
	// ������ �1��7 ������������ ����� �������� �0 �� ���� ����������� ������������� 
	//		���������� �� �������� ���, �.�. � �������� 1 ��.


	// ����� �0 0xFACE � ��������� ������� ������
	struct sDataBurstType0
	{
		union
		{
			unsigned char RawDataBurst[];
			
			struct
			{
				short int StatusFlags;		// ����� ���������
				long int CyclesIndexAfterSwitchingOn;		// ������ ������ ������ ����� ����� ��������� (50 ������ � �������)
				
				// ����� ������ � ���
				// ������ G - ���������, 
			    //        A - ������������� 
			    //        T - ������������ 
			    //        M - ������������ 
			    //        P - �������� 
			    //        U - ���������� ����������
				long int Gx1; 
			 	long int Ax1; 
			 	long int Mx1; 
			 	long int Ucc1; 
			 	long int Gy1; 
			 	long int Ay1; 
			 	long int My1; 
			  	long int Patm; 
				long int Gz1; 
			 	long int Az1; 
				long int Mz1;
				long int Az2; 
				long int Tx1; 
				long int Ty1; 
				long int Tz1; 
				long int Tcom; 
				
				float CourseCompl;		// ���� �����������������
				
				float MagneticComponentX;		// ��������� ���������� ����� ��� X ������� ��  
				float MagneticComponentY;		// ��������� ���������� ����� ��� Y ������� ��  
				float MagneticComponentZ;		// ��������� ���������� ����� ��� Z ������� ��  
				float Pressure;				// �������� (��)  
				float Pitch;					// ������ (�������) (�� �90 �� 90)   
				float Roll;					// ���� (�������) (�� �180 �� 180)   
				float TrackAngle;				// ������� ���� (�������) (�� 0 �� 360)   
				float MagneticCourse;			// ��������� ���� (�������) (�� 0 �� 360)   
				float LatitudeHigh;			// ������ �� ���������� ����� (�������)  
				float LatitudeLow;				// ������ ����� ���������� ����� (�������)  
				float LongitudeHigh;			// ������� �� ���������� ����� (�������)  
				float LongitudeLow;			// ������� ����� ���������� ����� (�������)  
				float Velocity;				// �������� (�/�)  
				float Altitude;				// ������ (�)  
				float ConstG;					// ����� g (1 �� = 9.81 �/�2)  
				float ClimbingSpeed;			// �������� ������� (�/�)  
				float BarometricAltitude;		// ��������������� ������ (�)  
				long  int  CyclesIndexAfterNavigationStart;	// ������ ������ ������ ����� ����� ������ ���������  
				float ApparentAccelerationX;	// ��������� ��������� �� ��� X ������� �� (�/�2)  
				float ApparentAccelerationY;	// ��������� ��������� �� ��� Y ������� �� (�/�2)  
				float ApparentAccelerationZ;	// ��������� ��������� �� ��� Z ������� �� (�/�2)  
				float AngularVelocityX;		// ������� �������� ������ ��� X ������� ��(����/�)  
				float AngularVelocityY;		// ������� �������� ������ ��� Y ������� ��(����/�)  
				float AngularVelocityZ;		// ������� �������� ������ ��� Z ������� ��(����/�)  
				float VelocityProjectionEastX;	// �������� �������� �� ��������� ��� (X) (�/�)  
				float VelocityProjectionNorthY;	// �������� �������� �� �������� ��� (Y) (�/�)  
			};
		};	
	};
	

inline std::ostream& operator<< (std::ostream& s, sDataBurstType0 dbt)
{
	return s << "StatusFlags = " << dbt.StatusFlags << endl <<
			"CyclesIndexAfterSwitchingOn = " << dbt.CyclesIndexAfterSwitchingOn << endl <<
			"Gx1 = " << dbt.Gx1 << endl <<
			"Ax1 = " << dbt.Ax1 << endl <<
			"Mx1 = " << dbt.Mx1  << endl <<
			"Ucc1 = " << dbt.Ucc1<< endl <<
			"Gy1 = " << dbt.Gy1 << endl <<
			"Ay1 = " << dbt.Ay1 << endl <<
			"My1 = " << dbt.My1 << endl <<
			"Patm = " << dbt.Patm << endl <<
			"Gz1 = " << dbt.Gz1<< endl <<
			"Az1 = " << dbt.Az1<< endl <<
			"Mz1 = " << dbt.Mz1 << endl <<
			"Az2 = " << dbt.Az2 << endl <<
			"Tx1 = " << dbt.Tx1 << endl <<
			"Ty1 = " << dbt.Ty1 << endl <<
			"Tz1 = " << dbt.Tz1 << endl <<
			"Tcom = " << dbt.Tcom << endl <<
			"CourseCompl = " << dbt.CourseCompl<< endl <<
			"MagneticComponentX = " << dbt.MagneticComponentX<< endl <<
			"MagneticComponentY = " << dbt.MagneticComponentY<< endl <<
			"MagneticComponentZ = " << dbt.MagneticComponentZ<< endl <<
			"Pressure = " << dbt.Pressure << endl <<
			"Pitch = " << dbt.Pitch << endl <<
			"Roll = " << dbt.Roll<< endl <<
			"TrackAngle = " << dbt.TrackAngle << endl <<
			"MagneticCourse = " << dbt.MagneticCourse << endl <<
			"LatitudeHigh = " << dbt.LatitudeHigh << endl <<
			"LatitudeLow = " << dbt.LatitudeLow << endl <<		
			"LongitudeHigh = " << dbt.LongitudeHigh << endl <<
			"LongitudeLow = " << dbt.LongitudeLow << endl <<
			"Velocity = " << dbt.Velocity << endl <<
			"Altitude = " << dbt.Altitude << endl <<
			"ConstG = " << dbt.ConstG << endl <<	
			"ClimbingSpeed = " << dbt.ClimbingSpeed << endl <<
			"BarometricAltitude = " << dbt.BarometricAltitude << endl <<
			"CyclesIndexAfterNavigationStart = " << dbt.CyclesIndexAfterNavigationStart<< endl <<
			"ApparentAccelerationX = " << dbt.ApparentAccelerationX<< endl <<
			"ApparentAccelerationY = " << dbt.ApparentAccelerationY<< endl <<
			"ApparentAccelerationZ = " << dbt.ApparentAccelerationZ<< endl <<
			"AngularVelocityX = " << dbt.AngularVelocityX<< endl <<
			"AngularVelocityY = " << dbt.AngularVelocityY<< endl <<
			"AngularVelocityZ = " << dbt.AngularVelocityZ<< endl <<
			"VelocityProjectionEastX = " << dbt.VelocityProjectionEastX<< endl <<
			"VelocityProjectionNorthY = " << dbt.VelocityProjectionNorthY<< endl;
}


	//����� �1 0xA511 � ��������� ������� ������
	struct sDataBurstType1
	{
		union
		{
			unsigned char RawDataBurst[];
			
			struct
			{
				float LatitudeRMCHigh;		// ������ RMC �� ���������� ����� (�������)  
				float LatitudeRMCLow;		// ������ RMC ����� ���������� ����� (�������)  
				float LongitudeRMCHigh;		// ������� RMC �� ���������� ����� (�������)  
				float LongitudeRMCLow;		// ������� RMC ����� ���������� ����� (�������)  
				float VelocityRMC;			// �������� RMC (�/�)  
				float TrackAngleRMC;		// ������� ���� RMC (�����������  ��������)  
				short int PDOP_GSA;			// GSA PDOP (���������� �� 100)   
				short int HDOP_GSA;			// GSA HDOP (x 100)   
				short int VDOP_GSA;			// GSA VDOP (x 100)   
				short int StatusFlags;		// ����� ��������� (�� ��, ��� � ���� 2 ������ �0)  
			};
		};
	};





	// ����� �2 0xA522 � ��������� ������� ������  
	struct sDataBurstType2
	{
		union
		{
			unsigned char RawDataBurst[];
			
			struct
			{
				float LatitudeGGAHigh; 	// ������ GGA �� ���������� ����� (�������)  
				float LatitudeGGALow; 	// ������ GGA ����� ���������� ����� (�������)  
				float LongitudeGGAHigh; // ������� GGA �� ���������� ����� (�������)  
				float LongitudeGGALow; 	// ������� GGA ����� ���������� ����� (�������)  
				float HDOPGGA; 			//  HDOP   
				float AltitudeGGA; 		//  ������ GGA (�)  
				float UTC_GGA;			// GGA UTC ����� � ��������  
				long int DateRMC;		// RMC ���� � ������������� �������, ������������ �� ���� ������� ����: ������ 
			};
		};
	};




	//����� �3 0xA533 � ��������� ������� ������ 
	struct sDataBurstType3
	{
		union
		{
			unsigned char RawDataBurst[];
			
			struct
			{  
				float UTC_RMC;							// RMC  UTC ����� � ��������  
				float MagneticDeclinationRMC;			// RMC ��������� ���������  
				short int SatellitesDescriptorsGSA[12];	// ������ ��������������� ��������� ID[12] �� ��������� GSA 
			};
		};
	};

 
	// ����� �4 0xA544 � ��������� ������� ������  
	struct sDataBurstType4
	{
		union
		{
			unsigned char RawDataBurst[];
			
			struct
			{  
				short int SatelitesAmountGGA; 			// ���-�� ��������� GGA   
				short int HDIFF_GGA;					// GGA HDIFF � ������� ����� ����� ������� � ���������� WGS84 
				short int SatelitesViewAmount;			// ���-�� ������������ ��������� GSV   
				short int SatellitesDescriptorsGSV[12];	// ������ ��������������� ��������� ID[12] GSV   
				short int DeviceRegime;					// ����� ������ ����� 0 � ���������� �����; 1 � ���������� �����; 2 � ����������� ����� 
			};
		};
	};
 
 
	// ����� �5 0xA555 � ��������� ������� ������  
	struct sDataBurstType5
	{
		union
		{
			unsigned char RawDataBurst[];
			
			struct
			{   
				short int ELEV_GSV[12];		//  ������ ELEV[12] ����� ���������� ��������� GSV (�������) 
				short int RFU[4];		// ������ z[4], ���������������  
			};
		};
	};
 

	// ����� �6 0xA566 � ��������� ������� ������
	struct sDataBurstType6
	{
		union
		{
			unsigned char RawDataBurst[];
			
			struct
			{    
				short int AZIM_GSV[12];	// ������ AZIM[12] �������� ��������� GSV (�������) 
				short int RFU[4];		// ������  z[4] , ���������������  
			};
		};
	};
 
	// ����� �7 0xA577 � ��������� ������� ������  
	struct sDataBurstType7
	{
		union
		{
			unsigned char RawDataBurst[];
			
			struct
			{  
				short int SN_GSV[12];	// ������ SN[12] ��������� ������/��� GSV (dB Hz) 
				short int RFU[4];		// ������  z[4] , ��������������� 
			};
		};
	}; 
	
	
	
//	// ����� ���������� ������ - ������ ������ ��������  
//	struct sControlBurstAlignment
//	{
//		static const unsigned short int signature = 0xA551;  	// ��������� ��������� 
//		static const unsigned short int id = 0x1111;			// �������������
//		float duration;  								// ����� �������� � �������� (�� ����� 5.0�)
//		float regime;									// ������� ���� ����� ������ �����
//		// �������� ����� � ���� ������ ������ �����: 
//		//  ������ ��� ������� ���� ���������� ����� ������ �����: 
//		// ��� �������� ������ 0 ���� ��������� � ����� �����������. 
//		// ��� �������� ������ 1 ���� ��������� � ����� �����������. 
//		// ��� �������� ������ 2 ���� ��������� � ����� ���������. 
//		//  ���������  ���  ����  ����������  �����  ������  � �������������  �  ������ ����������� � (���  ������� 
//		// ������ ���������, �������� � ������ ���� ������ ���� ����� 0): 
//		// ��� �������� ������ 0 ���� �� ���������� �������� �� ���������� ����� � �� ������������ �� ����. 
//		// ��� �������� ������ 1 ���� ���������� �������� �� ���������� ����� � �� ������������ �� ����. 
//		// ��� �������� ������ 2 ���� �� ���������� �������� �� ���������� �����, �� ������������ �� ����. 
//		// ��� �������� ������ 3 ���� ���������� �������� �� ���������� ����� � ������������ �� ����.								
//	}; 
	
	// ����� ���������� ������ - ��������� ��������������� ������   
	struct sControlBurstBarometricAltitudeCorrection
	{
		static const unsigned short int signature = 0xA551;  	// ��������� ��������� 
		static const unsigned short int id = 0x2222;			// �������������
		float current_altitude;							// ������� ������ (�����)
		float zero;										// ���� (0.0)				
	};


#pragma pack () // �������������� ��������� ������������ �� ���������

// ����� ��� ������ StatusFlags ������ 0 (sDataBurstType0)
// � ������������ ��������� �������� ������ �� �����
enum EStatusMasks
{
	// 1 - GPS ����� ��������� ���������� / 0 - GPS ����� ���������� � ����������� ���������
	EStatusMaskGPSGap = 0x0001,
	// 1 -�������� �������� ��������� ��������� / 0 - Move 
	EStatusMaskStop = 0x0002,
	// ��� ������ �������� GPS �� RMC ������; 1 - RMC_Valid, 0 - RMC_Invalid
	EStatusMaskRMC_Valid = 0x0004,
	// GGA qual value (��� ������ �������� GPS �� GGA ������)
	EStatusMaskGGAQualValue = 0x0030,
	// 1 - Nav (���� � ������ ���������) / 0 - Align (���� � ������ �������� (�� �������))
	EStatusMaskNav = 0x0100,
	// GSA CALCMODE (��� ������ �������� GPS �� GSA ������)
	EStatusMaskGSA_CALCMODE = 0x3000
	
};

enum EDataBurstType
{
	EDataBurstType0 = 01,
	EDataBurstType1 = 02,
	EDataBurstType2 = 04,
	EDataBurstType3 = 010,
	EDataBurstType4 = 020,
	EDataBurstType5 = 040,
	EDataBurstType6 = 0100,
	EDataBurstType7 = 0200,
	EDataBurstTypeUnknown = 0400
};


// ������� ���������� ����������� ����� ��� ��������� �������.
// 	"�����������  �����  �����������  �����������  ��������  �������� XOR  ��  ����  �����,
// 	�������������� � ���� ������� 16 ������ �������".
inline unsigned short int checksum( const unsigned char* bytesp, int size )
{
	unsigned short int two_bytes = 0;
	unsigned short int checksum_calc = 0;
	for(int i=0; i<=size-2; i+=2 )
	{
		//two_bytes = ((unsigned short int)(*(bytesp+i)) << 8) + (unsigned short int)(*(bytesp+i+1));
		memcpy(&two_bytes, (bytesp+i), 2);
		checksum_calc ^= two_bytes;
	}
	return checksum_calc;
}


using namespace Eigen;

class CCompanavHandler
{
public:
	CCompanavHandler();
	virtual ~CCompanavHandler();
	
	int InitCompanavHandlerThread(void* (*start_routine)(void* ));
	
	
	// ������� ������ ����������� ������ (������ �������� �� ������� ����������) - �������� ���� 
	// �� ����� ��������� ��� �������� ������;
	// �������� ������� ������� ��������������� ������� (������� ��������� �����. ����)
	//	�� �����:
	//	����� ������, ���������� �� ��, ����� ����������� ����� ���������
	// ���������� 1 ��� �������� ����������, 0 ��� ������
	bool GetNecessaryData(int what_to_read);
	
	// ������� ������ _���� ������_ ����� �� ����� (������� ����� ����������� ������, ������� ������ ������ �� ������� )
	// ��������� ������������ ������ (�� ����������� �����, ���� ������ �����������, ������ ���������),
	// ���������� ��� ���, ��������� ����� � ��������� ��������������� ����� ������ ���������
	// ���������� ��� ������������ ������
	EDataBurstType ReadAndAnalyseDataBurst();
	
	// �������� ����������� �������
	bool ControlAlignmentStart(float duration, float regime);
    bool ControlAlignmentStartWithConfirmation(float duration, float regime);
	bool ControlBarometricAltitudeCorrection(sControlBurstBarometricAltitudeCorrection);


	sDataBurstType0 LastDataBurstType0;
	sDataBurstType1 LastDataBurstType1;
	sDataBurstType2 LastDataBurstType2;
	sDataBurstType3 LastDataBurstType3;
	sDataBurstType4 LastDataBurstType4;
	sDataBurstType5 LastDataBurstType5;
	sDataBurstType6 LastDataBurstType6;
	sDataBurstType7 LastDataBurstType7;
	
	
//	XYZ start_point;
	bool start_point_set;
	
	Vector3d start_point;

	int set_start_point(double lat, double lon);
//	int set_init_angle(double ang)


	Vector3d geoToLocal(double lat, double lon);
	//Vector3d MetersFromLatLong(double lat, double lon, double height);


	int set_lat_long_logging(bool on, char* msg);	// ��������/��������� ������� ����� ������������� ������
	int log_current_lat_long();			// �������� � ���� ������� ������ � �������
	int empty_lat_long_log();			// �������� ���� navi_data.txt

//	int n = 5;
	double x_h[5];
	double y_h[5];
	double psi_h[5];
	double vx_h[5];
	double wy_h[5];


	 
private:
	// ������ ��� ����������� ������������� ������� � ����� 
	// (����� �� ���������� ���, ��� ������ ��������� ����� ��������, � ��������� ������� ������ � �.�.)
	pthread_mutex_t mutex;
	
	pthread_t m_Thread_id;	//�� ������ 
	pthread_attr_t m_tattr; //��������� ������
	
	int fd;	// �������� ���������� �����
	
	FILE *log_latlong_fp_htm;	// ����, � ������� ����� �������� �����, ������ � �������
	FILE *log_latlong_fp_txt;	// ����, � ������� ����� �������� �����, ������ � �������
	bool log_latlong; 	// ������ � ���-���� ������ � �������

	
};

#endif /*CCOMPANAVHANDLER_H_*/
