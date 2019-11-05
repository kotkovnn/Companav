#include "CCompanavHandler.h"
#include <errno.h>
#include "CLog.h"

#include <unistd.h>
#include <sys/timeb.h>
#include <time.h>

using namespace std;

extern CLog g_log;

CCompanavHandler::CCompanavHandler():m_Thread_id(0), log_latlong(false)
{
	// инициализация мутекса
	pthread_mutexattr_t attr;
	pthread_mutexattr_init( &attr );
	pthread_mutex_init(&mutex, &attr );
	
	
	empty_lat_long_log();
	start_point(0) = 47.224290;
	start_point(2) = 38.936169;

	for(int i=0;i++;i<4)
	{
		wy_h[i]=0;
	}
}

CCompanavHandler::~CCompanavHandler()
{
}


bool CCompanavHandler::GetNecessaryData(int what_to_read)
{
	while( what_to_read != 0 )
	{
		EDataBurstType DB_read = ReadAndAnalyseDataBurst();
		what_to_read &= ~DB_read;
		
		//g_Log.AddEntry(LOG_INFO, "what_to_read = %i", what_to_read);
	}
	
		
	return EXIT_SUCCESS;
}


EDataBurstType CCompanavHandler::ReadAndAnalyseDataBurst()
{
	unsigned char buf[1024];	
	memset(&buf, 0, sizeof(buf));
	
	EDataBurstType CurrentDataBurstType = EDataBurstTypeUnknown; 
	
	
	pthread_mutex_lock( &mutex );		
		
 		if(readcond(fd, buf+1, 1, 1, 0, 100)<=0)		// читаем кандидата в первый байт сигнатуры (в начале while он будет сдвинут)
 		{
 			g_Log.AddEntry(LOG_ERR, "Error reading signature1 from CompanavHandler port");
 		}
 		
		unsigned char chksumbuf[2];
		
		bool DataBurstReceived = false;			// принят ли какой-либо пакет (считан, проверена контрольная сумма)
		void* dist_structp = NULL;				// указатель на структуру, в которую нужно записывать данные (зависит от типа пакета, определяется после чтения сигнатуры)
		unsigned int dist_struct_size = 0; 		// размер этой структуры 
		
		
		while( !DataBurstReceived )
		{			
			// флаг, означающий, что получена сигнатура, соответствующая какому-либо пакету
			// сбрасывается в switch, если не один case не подошел
			bool have_signature = true;			
			
			buf[0] = buf[1];		// кандидата во второй байт сигнатуры пробуем в качестве первого
			if(readcond(fd, buf+1, 1, 1, 0, 100)<=0)		// читаем кандидата во второй байт сигнатуры
			{
				g_Log.AddEntry(LOG_ERR, "Error reading signature from CompanavHandler port");
			}
			
			// получаем число, которое будем проверять в качестве сигнатуры пакета
			unsigned short int signcand;
			memcpy(&signcand, buf, 2);
			
			CurrentDataBurstType = EDataBurstTypeUnknown;
			switch( signcand )
			{
				case 0xFACE:		// пакет 0 
				{
					//g_Log.AddEntry(LOG_INFO, "Have packet0 FACE signature");
					
					CurrentDataBurstType = EDataBurstType0;	
					dist_structp = &LastDataBurstType0;
					dist_struct_size = sizeof(sDataBurstType0);		 						
				}
				break;
				case 0xA511:		// пакет 1
				{
					//g_Log.AddEntry(LOG_INFO, "Have packet1 0xA511 signature");
					
					CurrentDataBurstType = EDataBurstType1;						
					dist_structp = &LastDataBurstType1;
					dist_struct_size = sizeof(sDataBurstType1);												
				}
				break;
				case 0xA522:		// пакет 2
				{
					//g_Log.AddEntry(LOG_INFO, "Have packet2 0xA522 signature");
					
					CurrentDataBurstType = EDataBurstType2;						
					dist_structp = &LastDataBurstType2;
					dist_struct_size = sizeof(sDataBurstType2);
				}
				break;
				case 0xA533:		// пакет 3
				{
					//g_Log.AddEntry(LOG_INFO, "Have packet3 0xA533 signature");
					
					CurrentDataBurstType = EDataBurstType3;						
					dist_structp = &LastDataBurstType3;
					dist_struct_size = sizeof(sDataBurstType3);											
				}
				break;
				case 0xA544:		// пакет 4
				{
					//g_Log.AddEntry(LOG_INFO, "Have packet4 0xA544 signature");
					
					CurrentDataBurstType = EDataBurstType4;
					dist_structp = &LastDataBurstType4;
					dist_struct_size = sizeof(sDataBurstType4);
				}
				break;
				case 0xA555:		// пакет 5
				{
					//g_Log.AddEntry(LOG_INFO, "Have packet5 0xA555 signature");
					
					CurrentDataBurstType = EDataBurstType5;
					dist_structp = &LastDataBurstType5;
					dist_struct_size = sizeof(sDataBurstType5);
				}
				break;
				case 0xA566:		// пакет 6
				{
					//g_Log.AddEntry(LOG_INFO, "Have packet6 0xA566 signature");
					
					CurrentDataBurstType = EDataBurstType6;
					dist_structp = &LastDataBurstType6;
					dist_struct_size = sizeof(sDataBurstType6);
				}
				break;
				case 0xA577:		// пакет 7
				{
					//g_Log.AddEntry(LOG_INFO, "Have packet7 0xA577 signature");
					
					CurrentDataBurstType = EDataBurstType7;
					dist_structp = &LastDataBurstType7;
					dist_struct_size = sizeof(sDataBurstType7);
				}
				break;
				default:
				{
					have_signature = false;
					CurrentDataBurstType = EDataBurstTypeUnknown;
				}
			}
				
			if(have_signature && CurrentDataBurstType!=EDataBurstTypeUnknown)
			{
				//g_Log.AddEntry(LOG_INFO, "packet n %i", CurrentDataBurstType);
				int bytesr = readcond(fd, buf+2, dist_struct_size, dist_struct_size, 0, 50 );
				if(bytesr != dist_struct_size)
				{
					g_Log.AddEntry(LOG_ERR, "Error reading DataBurst from CompanavHandler port");
				}
				
				if( readcond(fd, chksumbuf, 2, 2, 0, 50) != 2 )
				{
					g_Log.AddEntry(LOG_ERR, "Error reading checksum from CompanavHandler port");
				}
				
				unsigned short int chksum_r;
				memcpy(&chksum_r, chksumbuf, 2);
				
				if(chksum_r == checksum(buf, dist_struct_size+2) )
				{
					// g_Log.AddEntry(LOG_INFO, "Checksum OK");
					memcpy(dist_structp, buf+2, dist_struct_size);
					
					DataBurstReceived = true;												
				}
				else
				{
//					g_Log.AddEntry(LOG_WARNING, "Checksum error in packet id = %i", CurrentDataBurstType);
				}			
			}
		}
				
	
	pthread_mutex_unlock( &mutex );
	
	return CurrentDataBurstType;
}

// TODO
bool CCompanavHandler::ControlAlignmentStart(float duration, float regime)
{
	unsigned char buf[1024];
	buf[0] = 0x51;
	buf[1] = 0xA5;
	buf[2] = 0x11;
	buf[3] = 0x11;
	memcpy(buf+4, &duration, 4);
	memcpy(buf+8, &regime, 4);
	
	unsigned short int chksm = checksum(buf, 12);
	memcpy(buf+12, &chksm, 2);
	
	int br = write(fd, buf, 14);
	
	//g_Log.AddEntry(LOG_INFO, "br = %i", br);
	if(14 == br)
	{
		return EXIT_SUCCESS;
	}
	else
	{
		return EXIT_FAILURE;
	}
		
}

bool CCompanavHandler::ControlAlignmentStartWithConfirmation(float duration, float regime)
{
	GetNecessaryData( EDataBurstType0 );
	sleep(1);
	cout << "allign flag before " << ((LastDataBurstType0.StatusFlags&EStatusMaskNav) == EStatusMaskNav) << endl;

	do
	{
		ControlAlignmentStart( duration, regime ) ;
		sleep(1);
		GetNecessaryData( EDataBurstType0 );
		
		cout << "allign flag in process " << ((LastDataBurstType0.StatusFlags&EStatusMaskNav) == EStatusMaskNav) << endl;
	}while((LastDataBurstType0.StatusFlags&EStatusMaskNav) == EStatusMaskNav);
	cout << "allign flag after " << ((LastDataBurstType0.StatusFlags&EStatusMaskNav) == EStatusMaskNav) << endl;
	
	return true;
}

// TODO
bool CCompanavHandler::ControlBarometricAltitudeCorrection(sControlBurstBarometricAltitudeCorrection ctrls)
{
	
	return ENOSYS;

}



int CCompanavHandler::InitCompanavHandlerThread(void* (*start_routine)(void* ))
{
	int cntr = 0;
	do
	{
		fd = open(PORT_COMPANAV, O_RDWR);
		if(fd<0)
		{
			g_Log.AddEntry(LOG_ERR, "Unable to open Companav serial port, retrying... %d",cntr);
			sleep(2);
		}
		cntr++;
	}while((fd < 0)&&(cntr<10));
	if(cntr==10)
	{
		g_Log.AddEntry(LOG_ERR, "Unable to open Companav serial port, TERMINATING Companav thread");
		return -1;
	}
		
	// Configure the serial port. The serial port is configured such
	// that the baud rate is 115200 with no parity checking, 8 bit data,
	// and one stop bit.
	
	struct termios termio;
	if (tcgetattr(fd, &termio))
	{
		g_Log.AddEntry(LOG_ERR, "Cannot configure CompanavHandler COM port");
		//exit(1);
	}

	cfsetispeed(&termio, B115200);
	cfsetospeed(&termio, B115200);
	// флаги режима ввода
	termio.c_iflag &= ~(ICRNL|INLCR);	// 
	termio.c_iflag |= IGNPAR;
	// флаги режима вывода
	termio.c_oflag &= ~(OPOST);
	// флаги режима управления
	termio.c_cflag &= ~(CSIZE|PARENB);
	termio.c_cflag |= CREAD|CS8|CLOCAL;
	// флаги локального режима
	termio.c_lflag &= ~(ECHO|ECHOE|ECHOK|ECHONL|ICANON|ISIG);
	termio.c_cflag &= ~(IHFLOW|OHFLOW);
	
	tcsetattr(fd, TCSANOW, &termio); 
	
	/* Throw away all data */
    tcflush( fd, TCIFLUSH|TCOFLUSH|TCIOFLUSH );
	

	g_Log.AddEntry(LOG_INFO, "CompanavHandler serial port initialized");
	log_current_lat_long();
	
	if(!m_Thread_id)
	{
		// Создание потока
		pthread_attr_init( &m_tattr);
		pthread_attr_setdetachstate(&m_tattr, PTHREAD_CREATE_DETACHED);
		pthread_create( &m_Thread_id, &m_tattr, start_routine, NULL );
		g_Log.AddEntry(LOG_INFO, "Companav Handler thread succsessfully created");
	}
	else g_Log.AddEntry(LOG_WARNING, "Companav Handler thread not created, already exists");
	
	return EXIT_SUCCESS;
}

// записать в файл текущие широту и долготу
int CCompanavHandler::log_current_lat_long()
{
	struct timeb timebuf;
    char *now;
	if(log_latlong)
	{
		if( log_latlong_fp_htm != NULL )
		{
			ftime( &timebuf );
    		now = ctime( &timebuf.time );
    		
	    	double lat = LastDataBurstType0.LatitudeHigh 
							+ LastDataBurstType0.LatitudeLow;
			double lon = LastDataBurstType0.LongitudeHigh 
							+ LastDataBurstType0.LongitudeLow;
			double alt = LastDataBurstType0.Altitude;
    		
//    		XYZ meters_coords = MetersFromLatLong(lat, lon, alt);
    		fprintf( log_latlong_fp_htm, "<tr>\n <td> %.19s </td> \n <td> %f </td> \n <td> %f </td> \n <td> %f </td> \n<td> %f </td> \n<td> %f </td> \n<td> %f </td> \n<td> %f </td> \n<td> %f </td> \n<td> %f </td> \n<td> %f </td> \n<td> %f </td> \n<td> %f </td> \n<td> %f </td> \n<td> %f </td> \n<td> %f </td> \n<td> %f </td> \n <td> %f </td>\n <td> %f </td> \n <td> %f </td> \n </tr>\n",
    		 	now,		// время
    		 	lon, 	// долгота
    		 	lat,		// широта    		 	
				alt,			// Высота (м)  
				
//				meters_coords.X - start_point.X,
//			 	meters_coords.Z - start_point.Z,
//			 	meters_coords.Y - start_point.Y,

				
				LastDataBurstType0.CourseCompl,
				LastDataBurstType0.Pitch,
				LastDataBurstType0.Roll,
				
				LastDataBurstType0.Velocity,			// Скорость (м/с)  
				LastDataBurstType0.ClimbingSpeed,		// Скорость подъёма (м/с)
				LastDataBurstType0.VelocityProjectionEastX,	// Проекция скорости на восточную ось (X) (м/с)  
				LastDataBurstType0.VelocityProjectionNorthY,	// Проекция скорости на северную ось (Y) (м/с)
				
				LastDataBurstType0.ApparentAccelerationX,	// Кажущееся ускорение по оси X связной СК (м/с2)  
				LastDataBurstType0.ApparentAccelerationY,	// Кажущееся ускорение по оси Y связной СК (м/с2)  
				LastDataBurstType0.ApparentAccelerationZ,	// Кажущееся ускорение по оси Z связной СК (м/с2)  
				
				LastDataBurstType0.AngularVelocityX,		// Угловая скорость вокруг оси X связной СК(град/с)  
				LastDataBurstType0.AngularVelocityY,		// Угловая скорость вокруг оси Y связной СК(град/с)  
				LastDataBurstType0.AngularVelocityZ	 ); 	// Угловая скорость вокруг оси Z связной СК(град/с)  
    		 	
    		fprintf( log_latlong_fp_txt, "%.19s \t %f \t %f \n",
    		 	now,
    		 	LastDataBurstType0.LongitudeHigh + LastDataBurstType0.LongitudeLow,
    		 	LastDataBurstType0.LatitudeHigh + LastDataBurstType0.LatitudeLow);
   			
    		fflush( log_latlong_fp_htm );
    		fflush( log_latlong_fp_txt );	
		}
		else
		{
			g_Log.AddEntry(LOG_WARNING, "CompanavHandler: log_latlong is on but fp == NULL");
		}
	}
	return EXIT_SUCCESS;
}		

Vector3d CCompanavHandler::geoToLocal(double lat,double lon)
{
//
	/*float n,l,x,y;
	 n = 7;
	 l = (lon-(3+6*(n - 1)))/57.29577951;

     float B = lat*pi/180;

	x = 6367558.4968*B - sin(2*B)*(16002.8900 + 66.9607*pow(sin(B),2) + 0.3515*pow(sin(B),4) -
	- l*l*(1594561.25 + 5336.535*pow(sin(B),2) + 26.790*pow(sin(B),4) + 0.149*pow(sin(B),6) +
	+ l*l*(672483.4 - 811219.9*pow(sin(B),2) + 5420.0*pow(sin(B),4) - 10.6*pow(sin(B),6) +
	+ l*l*(278194 - 830174*pow(sin(B),2) + 572434*pow(sin(B),4) - 16010*pow(sin(B),6) +
	+ l*l*(109500 - 574700*pow(sin(B),2) + 863700*pow(sin(B),4) - 398600*pow(sin(B),6)  )))));

	y = (5 + 10*n)*105 +l*cos(B)*(6378245 + 21346.1415*pow(sin(B),2) + 107.1590*pow(sin(B),4) +
	+ 0.5977*pow(sin(B),6) + l*l*(1070204.16 - 2136826.66*pow(sin(B),2) + 17.98*pow(sin(B),4) - 11.99*pow(sin(B),6) +
	+ l*l*(270806 - 1523417*pow(sin(B),2) + 1327645*pow(sin(B),4) - 21701*pow(sin(B),6) +
	+ l*l*(79690 - 866190*pow(sin(B),2) + 1730360*pow(sin(B),4) - 945460*pow(sin(B),6) ))));


*/
double db = 75745.819010272; // на один градус
double dl = 111179.849121609;
	//fprintf(stdout,"%f %f \n",dB,dL);
	//перреводим в радианы
	//lat = lat*pi/180;
	//lon = lon*pi/180;

	// расччет кол-ва метров в радиане

	/*double f = 1/298.257223563;
	double e = sqrt(f*(2-f));
	double b0 = start_point(0);
	double l0 = start_point(2);

	double a = 6378245.0;
	double W = sqrt(1-(e*sin(b0))*(e*sin(b0)));
	double dB = (a*(1-e*e))/(W*W*W)*(lat-b0);//*(lat-start_point(0));
	double dL = (a*cos(b0))/W*(lon-l0);//*(lon-start_point(2));*/

	//fprintf(stdout,"x = %f y = %f \n",(lat-start_point(0))*db,(lon-start_point(2))*dl);
	//
	return Vector3d((lat-start_point(0))*db,0,(lon-start_point(2))*dl);
	//return Vector3f(0,0,0);
 	// считаем приращения от стартовой точки;
}
// очистить файл navi_data.txt
int CCompanavHandler::empty_lat_long_log()
{
	struct timeb timebuf;
    char *now;
	
	fclose(log_latlong_fp_htm);	
	log_latlong_fp_htm = fopen(LAT_LONG_LOG_HTM, "w+");
	
	fclose(log_latlong_fp_txt);	
	log_latlong_fp_txt = fopen(LAT_LONG_LOG_TXT, "w+");
   
	ftime( &timebuf );
	now = ctime( &timebuf.time );

	fprintf( log_latlong_fp_htm, "%.19s Logfile cleaned<br>\n",	now );
	fprintf( log_latlong_fp_txt, "%.19s Logfile cleaned\n",	now );		
	
	return EXIT_SUCCESS;
}	

// включить/выключить ведение логов навигационных данных
int CCompanavHandler::set_lat_long_logging(bool on, char* msg)
{
	struct timeb timebuf;
    char *now;
	if(on)
	{
		if(!log_latlong)
		{
			log_latlong_fp_htm = fopen(LAT_LONG_LOG_HTM, "a+");
			log_latlong_fp_txt = fopen(LAT_LONG_LOG_TXT, "a+");

    		ftime( &timebuf );
    		now = ctime( &timebuf.time );

    		fprintf( log_latlong_fp_htm, "%.19s Starting navi data logging...<br>\n", now);
            fprintf( log_latlong_fp_htm, "<table border=1 bordercolor=black cellspacing=0 cellpadding=5>");
            
            fprintf( log_latlong_fp_htm, "<tr> \n <th>time</th> \n <th>lon</th> \n <th>lat</th> \n ");
            fprintf( log_latlong_fp_htm, "<th>alt</th> \n <th>meters_coords.X</th> \n <th>meters_coords.Z</th> \n ");
            fprintf( log_latlong_fp_htm, "<th>meters_coords.Y</th> \n <th>CourseCompl</th> \n <th>Pitch</th> \n ");
            fprintf( log_latlong_fp_htm, "<th>Roll</th> \n <th>Velocity</th> \n <th>ClimbingSpeed</th> \n ");
            fprintf( log_latlong_fp_htm, "<th>VelocityProjectionEast</th> \n <th>VelocityProjectionNorth</th> \n <th>ApparentAccelerationX</th> \n ");
            fprintf( log_latlong_fp_htm, "<th>ApparentAccelerationY</th> \n <th>ApparentAccelerationZ</th> \n <th>AngularVelocityX</th> \n ");
            fprintf( log_latlong_fp_htm, "<th>AngularVelocityY</th> \n <th>AngularVelocityZ</th> \n    </tr> \n "); 		
            
            
            sprintf(msg, "%.19s Starting navi data logging\n", now);
            
            fprintf( log_latlong_fp_txt, "%.19s Starting navi data logging...\n", now);
            
            log_latlong = true;	
		}
		else
		{
			sprintf(msg, "Navi data logging is already on\n");
		}
		
	}
	else	
	{
		if(log_latlong)
		{
			ftime( &timebuf );
    		now = ctime( &timebuf.time );

			fprintf( log_latlong_fp_htm, "</table>");	
    		fprintf( log_latlong_fp_htm, "%.19s Navi data logging stoped.<br>\n", now);
    		
    		fprintf( log_latlong_fp_txt, "%.19s Navi data logging stoped.\n", now);
            
			fclose(log_latlong_fp_htm);
			log_latlong = false;
			sprintf( msg, "%.19s Navi data logging stoped\n", now);	
		}
		else
		{
			sprintf(msg, "Navi data logging is already off\n");
		}
		
	}
	
	return EXIT_SUCCESS;
}	

int CCompanavHandler::set_start_point(double lat, double lon)
{
		start_point(0) = lat;
		start_point(2)= lon;


	return EXIT_SUCCESS;

}









