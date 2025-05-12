/*
 * GPSFix.h
 *
 *  Created on: Jul 23, 2014
 *      Author: Cameron Karlsson
 *
 *  See the license file included with this source.
 */

#ifndef GPSFIX_H_
#define GPSFIX_H_

#include <cstdint>
#include <ctime>
#include <string>
#include <chrono>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <sstream>

namespace nmea {

	class GPSSatellite;
	class GPSAlmanac;
	class GPSFix;
	class GPSService;


	// =========================== GPS SATELLITE =====================================

	class GPSSatellite {
	public:
		GPSSatellite() :
			snr(0),
			prn(0),
			elevation(0),
			azimuth(0)
		{};

		//satellite data
		double snr;			// 0-99 dB
		uint32_t prn;		// id - 0-32
		double elevation;	// 0-90 deg
		double azimuth;		// 0-359 deg
		std::string toString();
		operator std::string();
	};





	// =========================== GPS TIMESTAMP =====================================

	// UTC time
	class GPSTimestamp {
		private:
			std::string monthName(uint32_t index);
		public:
			GPSTimestamp();
	
			int32_t hour;
			int32_t min;
			double sec;
	
			int32_t month;
			int32_t day;
			int32_t year;
	
			// Values collected directly from the GPS
			double rawTime;
			int32_t rawDate;
	
			time_t getTime();
	
			uint64_t getTimeMilliseconds();
	
			// Set directly from the NMEA time stamp
			// hhmmss.sss
			void setTime(double raw_ts);
	
			// Set directly from the NMEA date stamp
			// ddmmyy
			void setDate(int32_t raw_date);
	
			std::string toString();
		};



	// =========================== GPS ALMANAC =====================================


	class GPSAlmanac {
		friend GPSService;
	private:
		std::string talkerSignalId;
		uint32_t lastPage;
		uint32_t totalPages;
		uint32_t processedPages;
		void clear();			//will remove all information from the satellites
		void addSatellite(GPSSatellite sat);
	public:
		GPSAlmanac(std::string id) :
			lastPage(0),
			totalPages(0),
			processedPages(0),
			visibleSatelites(0)
		{
			talkerSignalId = id;
		};

		//mapped by prn
		std::vector<GPSSatellite> satellites;
		uint32_t visibleSatelites;
		GPSTimestamp lastUpdate;
		double percentComplete();
	};



	// =========================== GPS FIX =====================================

	class GPSFix {
		friend GPSService;

	private:
		GPSTimestamp lastKnownEpoch;
		bool haslock;
		bool setlock(bool b);		//returns true if lock status **changed***, false otherwise.
	public:

		GPSFix();
		virtual ~GPSFix();

		std::unordered_map<std::string, nmea::GPSAlmanac> almanacTable;

		char status;		// Status: A=active, V=void (not locked)
		uint8_t type;		// Type: 1=none, 2=2d, 3=3d
		uint8_t quality;	// Quality: 
							//    0 = invalid
							//    1 = GPS fix (SPS)
							//    2 = DGPS fix
							//    3 = PPS fix
							//    4 = Real Time Kinematic (RTK)
							//    5 = Float RTK
							//    6 = estimated (dead reckoning) (2.3 feature)

		GPSTimestamp GSA_epoch; //type->lock, dilution, horizontalDilution, verticalDilution
		
		double dilution;					// Combination of Vertical & Horizontal
		double horizontalDilution;			// Horizontal dilution of precision, initialized to 100, best =1, worst = >20
		double verticalDilution;			// Vertical is less accurate

		GPSTimestamp GST_epoch; //rmsDeviation, semiMajorDeviation, semiMinorDeviation, semiMajorOrient, latitudeDeviation, longitudeDeviation, altitudeDeviation
		double rmsDeviation;				// Root mean square value of the standard deviation of the range inputs to the navigation process.
		double semiMajorDeviation;			// Standard deviation of semi-major axis of error ellipse, in meters 
		double semiMinorDeviation;			// Standard deviation of semi-minor axis of error ellipse, in meters
		double semiMajorOrient;				// Orientation of semi-major axis of error ellipse (degrees from true north)
		double latitudeDeviation;			// Standard deviation of latitude error (m)
		double longitudeDeviation;			// Standard deviation of longitude error (m)
		double altitudeDeviation;			// Standard deviation of altitude error (m)

		GPSTimestamp GGA_epoch; //latitude, longitude, altitude, quality->lock, diffAge, diffStation, trackingSatellites
		double altitude;		// meters
		double latitude;		// degrees N
		double longitude;		// degrees E
		
		GPSTimestamp RMC_epoch; //longitude, longitude, status->lock, speed, travelAngle
		double speed;			// km/h
		double travelAngle;		// degrees true north (0-360)
		uint32_t trackingSatellites;

		double diffAge;
		std::string diffStation;

		bool locked();
		double horizontalAccuracy();
		double verticalAccuracy();
		bool hasEstimate();
		
		std::chrono::seconds timeSince(GPSTimestamp timestamp);	// Returns seconds difference from last timestamp and right now.
		std::chrono::seconds timeSince(time_t now, time_t then);	// Returns seconds difference from then to now

		std::string toString();
		operator std::string();

		static std::string travelAngleToCompassDirection(double deg, bool abbrev = false);

		uint32_t visibleSatellites();
		double averageSNR();
		double minSNR();
		double maxSNR();
		double almanacPercentComplete();
	};

}

#endif /* GPSFIX_H_ */
