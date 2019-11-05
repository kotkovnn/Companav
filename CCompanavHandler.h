/* Модуль взаимодействия с интегрированной навигационной системой "Компанав-2"
 * Назначение:
 * 	- прием данных от устройства в соответствии с бинарным протоколом,
 * 	- запись их в соответствующие структуры,
 * 	- преобразование этих данных (широта-долгота -> линейные координаты) - здесь вопрос,
 * 	- обновление соответствующих полей в g_Robot,
 * 	- управление устройством (режим выставки и др.).
 * Состав:
 * 	- класс CCompanavHandler (основные поля данных и функции),
 * 	- поток, осуществляющий циклическое получение данных,
 * 	- ряд объявлений структур для хранения поступающих данных.
*/

// TODO: блокировки чтения-записи 

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


// отключение выравнивания полей структуры в памяти
// + позволит копировать массив, поступающий от устройства непосредственно в структуру
// - может замедлить скорость обращения к структуре
// ! в двух разных единицах компиляции не должно использоваться разное выравнивание
//    для одних и тех же структур данных 
// ! проверить, работает ли это в других версиях компилятора
// альтернатива - "ручное копирование" каждого элемента в соответствии с его длиной
// другой, специфичный для gcc, способ: __attribute__((aligned (n))); 
#pragma pack (1) 
	
	// типы пакетов данных, поступающих от устройства:
	// Пакет  №0  отправляется  потребителю  навигационной  информации  с  частотой 50  Гц. 
	// Пакеты №1–№7 отправляются между пакетами №0 по мере поступления навигационной 
	//		информации от приёмника СНС, т.е. с частотой 1 Гц.


	// Пакет №0 0xFACE – заголовок данного пакета
	struct sDataBurstType0
	{
		union
		{
			unsigned char RawDataBurst[];
			
			struct
			{
				short int StatusFlags;		// флаги состояния
				long int CyclesIndexAfterSwitchingOn;		// индекс циклов работы блока после включения (50 единиц в секунде)
				
				// Сырые данные с АЦП
				// символ G - гироскопы, 
			    //        A - акселерометры 
			    //        T - термодатчики 
			    //        M - магнитометры 
			    //        P - давление 
			    //        U - внутреннее напряжение
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
				
				float CourseCompl;		// Курс комплексированный
				
				float MagneticComponentX;		// Магнитная компонента вдоль оси X связной СК  
				float MagneticComponentY;		// Магнитная компонента вдоль оси Y связной СК  
				float MagneticComponentZ;		// Магнитная компонента вдоль оси Z связной СК  
				float Pressure;				// Давление (Па)  
				float Pitch;					// Тангаж (градусы) (от –90 до 90)   
				float Roll;					// Крен (градусы) (от –180 до 180)   
				float TrackAngle;				// Путевой угол (градусы) (от 0 до 360)   
				float MagneticCourse;			// Магнитный курс (градусы) (от 0 до 360)   
				float LatitudeHigh;			// Широта до десятичной точки (градусы)  
				float LatitudeLow;				// Широта после десятичной точки (градусы)  
				float LongitudeHigh;			// Долгота до десятичной точки (градусы)  
				float LongitudeLow;			// Долгота после десятичной точки (градусы)  
				float Velocity;				// Скорость (м/с)  
				float Altitude;				// Высота (м)  
				float ConstG;					// Число g (1 ед = 9.81 м/с2)  
				float ClimbingSpeed;			// Скорость подъёма (м/с)  
				float BarometricAltitude;		// Барометрическая высота (м)  
				long  int  CyclesIndexAfterNavigationStart;	// Индекс циклов работы блока после начала навигации  
				float ApparentAccelerationX;	// Кажущееся ускорение по оси X связной СК (м/с2)  
				float ApparentAccelerationY;	// Кажущееся ускорение по оси Y связной СК (м/с2)  
				float ApparentAccelerationZ;	// Кажущееся ускорение по оси Z связной СК (м/с2)  
				float AngularVelocityX;		// Угловая скорость вокруг оси X связной СК(град/с)  
				float AngularVelocityY;		// Угловая скорость вокруг оси Y связной СК(град/с)  
				float AngularVelocityZ;		// Угловая скорость вокруг оси Z связной СК(град/с)  
				float VelocityProjectionEastX;	// Проекция скорости на восточную ось (X) (м/с)  
				float VelocityProjectionNorthY;	// Проекция скорости на северную ось (Y) (м/с)  
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


	//Пакет №1 0xA511 – заголовок данного пакета
	struct sDataBurstType1
	{
		union
		{
			unsigned char RawDataBurst[];
			
			struct
			{
				float LatitudeRMCHigh;		// Широта RMC до десятичной точки (градусы)  
				float LatitudeRMCLow;		// Широта RMC после десятичной точки (градусы)  
				float LongitudeRMCHigh;		// Долгота RMC до десятичной точки (градусы)  
				float LongitudeRMCLow;		// Долгота RMC после десятичной точки (градусы)  
				float VelocityRMC;			// Скорость RMC (м/с)  
				float TrackAngleRMC;		// Путевой угол RMC (направление  скорости)  
				short int PDOP_GSA;			// GSA PDOP (умноженный на 100)   
				short int HDOP_GSA;			// GSA HDOP (x 100)   
				short int VDOP_GSA;			// GSA VDOP (x 100)   
				short int StatusFlags;		// Флаги состояния (те же, что в поле 2 пакета №0)  
			};
		};
	};





	// Пакет №2 0xA522 – заголовок данного пакета  
	struct sDataBurstType2
	{
		union
		{
			unsigned char RawDataBurst[];
			
			struct
			{
				float LatitudeGGAHigh; 	// Широта GGA до десятичной точки (градусы)  
				float LatitudeGGALow; 	// Широта GGA после десятичной точки (градусы)  
				float LongitudeGGAHigh; // Долгота GGA до десятичной точки (градусы)  
				float LongitudeGGALow; 	// Долгота GGA после десятичной точки (градусы)  
				float HDOPGGA; 			//  HDOP   
				float AltitudeGGA; 		//  Высота GGA (м)  
				float UTC_GGA;			// GGA UTC время в секундах  
				long int DateRMC;		// RMC Дата в целочисленном формате, составленном из цифр текущей даты: ддммгг 
			};
		};
	};




	//Пакет №3 0xA533 – заголовок данного пакета 
	struct sDataBurstType3
	{
		union
		{
			unsigned char RawDataBurst[];
			
			struct
			{  
				float UTC_RMC;							// RMC  UTC время в секундах  
				float MagneticDeclinationRMC;			// RMC магнитное склонение  
				short int SatellitesDescriptorsGSA[12];	// Массив идентификаторов спутников ID[12] из протокола GSA 
			};
		};
	};

 
	// Пакет №4 0xA544 – заголовок данного пакета  
	struct sDataBurstType4
	{
		union
		{
			unsigned char RawDataBurst[];
			
			struct
			{  
				short int SatelitesAmountGGA; 			// Кол-во спутников GGA   
				short int HDIFF_GGA;					// GGA HDIFF – разница высот между геойдом и элиплойдом WGS84 
				short int SatelitesViewAmount;			// Кол-во обозреваемых спутников GSV   
				short int SatellitesDescriptorsGSV[12];	// Массив идентификаторов спутников ID[12] GSV   
				short int DeviceRegime;					// Режим работы блока 0 – Вертолётный режим; 1 – Поверочный режим; 2 – Авиационный режим 
			};
		};
	};
 
 
	// Пакет №5 0xA555 – заголовок данного пакета  
	struct sDataBurstType5
	{
		union
		{
			unsigned char RawDataBurst[];
			
			struct
			{   
				short int ELEV_GSV[12];		//  Массив ELEV[12] углов возвышения спутников GSV (градусы) 
				short int RFU[4];		// Массив z[4], зарезервировано  
			};
		};
	};
 

	// Пакет №6 0xA566 – заголовок данного пакета
	struct sDataBurstType6
	{
		union
		{
			unsigned char RawDataBurst[];
			
			struct
			{    
				short int AZIM_GSV[12];	// Массив AZIM[12] азимутов спутников GSV (градусы) 
				short int RFU[4];		// Массив  z[4] , зарезервировано  
			};
		};
	};
 
	// Пакет №7 0xA577 – заголовок данного пакета  
	struct sDataBurstType7
	{
		union
		{
			unsigned char RawDataBurst[];
			
			struct
			{  
				short int SN_GSV[12];	// Массив SN[12] отношений сигнал/шум GSV (dB Hz) 
				short int RFU[4];		// Массив  z[4] , зарезервировано 
			};
		};
	}; 
	
	
	
//	// Пакет управления блоком - запуск режима выставки  
//	struct sControlBurstAlignment
//	{
//		static const unsigned short int signature = 0xA551;  	// Заголовок сообщения 
//		static const unsigned short int id = 0x1111;			// Идентификатор
//		float duration;  								// Время выставки в секундах (не менее 5.0с)
//		float regime;									// Битовое поле Режим работы блока
//		// Значения битов в поле режима работы блока: 
//		//  Первые два младших бита определяют режим работы блока: 
//		// При значении равном 0 блок переходит в режим “Вертолётный”. 
//		// При значении равном 1 блок переходит в режим “Автономный”. 
//		// При значении равном 2 блок переходит в режим “Авиация”. 
//		//  Следующие  два  бита  определяют  режим  работы  с магнитометром  в  режиме “Автономный ” (при  запуске 
//		// режима “Авиация”, значение в данном поле должно быть равно 0): 
//		// При значении равном 0 блок не производит выставку по магнитному курсу и не демпфируется по нему. 
//		// При значении равном 1 блок производит выставку по магнитному курсу и не демпфируется по нему. 
//		// При значении равном 2 блок не производит выставку по магнитному курсу, но демпфируется по нему. 
//		// При значении равном 3 блок производит выставку по магнитному курсу и демпфируется по нему.								
//	}; 
	
	// Пакет управления блоком - Коррекция барометрической высоты   
	struct sControlBurstBarometricAltitudeCorrection
	{
		static const unsigned short int signature = 0xA551;  	// Заголовок сообщения 
		static const unsigned short int id = 0x2222;			// Идентификатор
		float current_altitude;							// Текущая высота (метры)
		float zero;										// Ноль (0.0)				
	};


#pragma pack () // восстановления параметра выравнивания по умолчанию

// маски для флагов StatusFlags пакета 0 (sDataBurstType0)
// в комментариях приведены значения флагов по маске
enum EStatusMasks
{
	// 1 - GPS выдаёт ошибочные координаты / 0 - GPS выдаёт координаты с достаточной точностью
	EStatusMaskGPSGap = 0x0001,
	// 1 -детектор движения определил остановку / 0 - Move 
	EStatusMaskStop = 0x0002,
	// код режима приёмника GPS из RMC пакета; 1 - RMC_Valid, 0 - RMC_Invalid
	EStatusMaskRMC_Valid = 0x0004,
	// GGA qual value (код режима приёмника GPS из GGA пакета)
	EStatusMaskGGAQualValue = 0x0030,
	// 1 - Nav (блок в режиме навигации) / 0 - Align (блок в режиме выставки (не двигать))
	EStatusMaskNav = 0x0100,
	// GSA CALCMODE (код режима приёмника GPS из GSA пакета)
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


// Функция вычисления контрольной суммы для байтового массива.
// 	"Контрольная  сумма  вычисляется  применением  бинарной  операции XOR  на  весь  пакет,
// 	представленный в виде массива 16 битных величин".
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
	
	
	// функция читает необходимые пакеты (список задается во входных параметрах) - работает пока 
	// не будут прочитаны все заданные пакеты;
	// вызывает функции разбора соответствующих пакетов (которые обновляют соотв. поля)
	//	на входе:
	//	набор флагов, отвечающих за то, какие предложения нужно прочитать
	// возвращает 1 при успешном выполнении, 0 при ошибке
	bool GetNecessaryData(int what_to_read);
	
	// функция читает _один первый_ пакет из порта (отсекая байты предыдущего пакета, который начали читать не сначала )
	// проверяет правильность чтения (по контрольной сумме, если считан неправильно, читает следующий),
	// определяет его тип, разбирает пакет и обновляет соответствующую этому пакету структуру
	// возвращает тип прочитанного пакета
	EDataBurstType ReadAndAnalyseDataBurst();
	
	// отправка управляющих пакетов
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


	int set_lat_long_logging(bool on, char* msg);	// включить/выключить ведение логов навигационных данных
	int log_current_lat_long();			// записать в файл текущие широту и долготу
	int empty_lat_long_log();			// очистить файл navi_data.txt

//	int n = 5;
	double x_h[5];
	double y_h[5];
	double psi_h[5];
	double vx_h[5];
	double wy_h[5];


	 
private:
	// мутекс для обеспечения эксклюзивного доступа к порту 
	// (чтобы не получилось так, что запрос отправлен одной функцией, а результат получен другой и т.д.)
	pthread_mutex_t mutex;
	
	pthread_t m_Thread_id;	//ИД потока 
	pthread_attr_t m_tattr; //Аттрибуты потока
	
	int fd;	// файловый дескриптор порта
	
	FILE *log_latlong_fp_htm;	// файл, в который будут писаться время, широта и долгота
	FILE *log_latlong_fp_txt;	// файл, в который будут писаться время, широта и долгота
	bool log_latlong; 	// писать в лог-файл широту и долготу

	
};

#endif /*CCOMPANAVHANDLER_H_*/
