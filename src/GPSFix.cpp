/*
 * GPSFix.cpp
 *
 *  Created on: Jul 23, 2014
 *      Author: Cameron Karlsson
 *
 *  See the license file included with this source.
 */

#include <nmeaparse/GPSFix.h>
#include <nmeaparse/GPSService.h>
#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>

using namespace std;
using namespace std::chrono;

using namespace nmea;



// ===========================================================
// ======================== GPS SATELLITE ====================
// ===========================================================

string GPSSatellite::toString(){
	stringstream ss;

	ss << "[PRN: " << setw(3) << setfill(' ') << prn << " "
		<< "  SNR: " << setw(3) << setfill(' ') << snr << " dB  "
		<< "  Azimuth: " << setw(3) << setfill(' ') << azimuth << " deg "
		<< "  Elevation: " << setw(3) << setfill(' ') << elevation << " deg  "
		<< "]";

	return ss.str();
}
GPSSatellite::operator std::string(){
	return toString();
}



// =========================================================
// ======================== GPS ALMANAC ====================
// =========================================================

void GPSAlmanac::clear(){
	lastPage = 0;
	totalPages = 0;
	processedPages = 0;
	visibleSatelites = 0;
	satellites.clear();
}
void GPSAlmanac::addSatellite(GPSSatellite sat){
	satellites.push_back(sat);
}
double GPSAlmanac::percentComplete(){
	if (totalPages == 0){
		return 0.0;
	}

	return ((double)processedPages) / ((double)totalPages) * 100.0;
}

// ===========================================================
// ===================== PUBX03 SATELLITE ====================
// ===========================================================

string PUBX03Satellite::toString(){
	stringstream ss;
	ss << "[svid: " << setw(3) << setfill(' ') << svid << " "
		<< "  C/N0: " << setw(3) << setfill(' ') << (int)cno << " dBHz  "
		<< "  Used: " << (usedInFix ? "YES" : "no ")
		<< "  Az: " << setw(3) << setfill(' ') << (int)azimuth << " deg "
		<< "  El: " << setw(3) << setfill(' ') << (int)elevation << " deg  "
		<< "  Lock: " << carrierLockTime << " s"
		<< "]";
	return ss.str();
}
PUBX03Satellite::operator std::string(){
	return toString();
}

// ===========================================================
// ====================== PUBX03 ALMANAC =====================
// ===========================================================

void PUBX03Almanac::clear(){
	satellites.clear();
}
void PUBX03Almanac::addSatellite(PUBX03Satellite sat){
	satellites.push_back(sat);
}

// ===========================================================
// ======================== GPS TIMESTAMP ====================
// ===========================================================


GPSTimestamp::GPSTimestamp(){
	hour = 0;
	min = 0;
	sec = 0;

	month = 1;
	day = 1;
	year = 1970;

	rawTime = 0;
	rawDate = 0;
};

// indexed from 1!
std::string GPSTimestamp::monthName(uint32_t index){
	if (index < 1 || index > 12){
		std::stringstream ss;
		ss << "[month:" << index << "]";
		return ss.str();
	}

	std::string names[] = {
		"January",
		"February",
		"March",
		"April",
		"May",
		"June",
		"July",
		"August",
		"September",
		"October",
		"November",
		"December"
	};
	return names[index - 1];
};

// Returns seconds since Jan 1, 1970. Classic Epoch time.
time_t GPSTimestamp::getTime() {
	struct tm t = { 0 };
	t.tm_year = year - 1900;	// This is year-1900, so 112 = 2012
	t.tm_mon = month - 1;		// month from 0:Jan
	t.tm_mday = day;
	t.tm_hour = hour;
	t.tm_min = min;
	t.tm_sec = (int)sec;
	return mktime(&t);
}

uint64_t GPSTimestamp::getTimeMilliseconds() {
	uint64_t seconds = (uint64_t)getTime();
	return seconds*1000 + (uint64_t) ((rawTime - (long)rawTime) * 1000);
}

void GPSTimestamp::setTime(double raw_ts){
	rawTime = raw_ts;

	hour = (int32_t)trunc(raw_ts / 10000.0);
	min = (int32_t)trunc((raw_ts - hour * 10000) / 100.0);
	sec = raw_ts - min * 100 - hour * 10000;
}

//ddmmyy
void GPSTimestamp::setDate(int32_t raw_date){
	rawDate = raw_date;
	// If uninitialized, use posix time.
	if(rawDate == 0) {
		month = 1;
		day = 1;
		year = 1970;
	}
	else {
		day = (int32_t)trunc(raw_date / 10000.0);
		month = (int32_t)trunc((raw_date - 10000 * day) / 100.0);
		year = raw_date - 10000 * day - 100 * month + 2000;
	}
}

std::string GPSTimestamp::toString(){
	std::stringstream ss;
	ss << hour << "h " << min << "m " << sec << "s" << "  " << monthName(month) << " " << day << " " << year;
	return ss.str();
};







// =====================================================
// ======================== GPS FIX ====================
// =====================================================

GPSFix::GPSFix() {

	quality = 0;	// Searching...
	status = 'V';	// Void
	type = 1;		// 1=none, 2=2d, 3=3d

	haslock = 0;

	dilution = 0;		
	horizontalDilution = 0;		// Horizontal - Best is 1, >20 is terrible, so 0 means uninitialized
	verticalDilution = 0;	
	latitudeDeviation = 0;
	longitudeDeviation = 0;
	altitudeDeviation = 0;
	latitude = 0;	
	longitude = 0;	
	speed = 0;
	travelAngle = 0;
	altitude = 0;

	trackingSatellites = 0;
}

GPSFix::~GPSFix() {
	// TODO Auto-generated destructor stub
}

// Returns the duration since the Host has received information
seconds GPSFix::timeSince(GPSTimestamp timestamp){
	time_t now = time(NULL);
	time_t then = timestamp.getTime();
	return timeSince(now, then);
}

seconds GPSFix::timeSince(time_t now, time_t then){
	uint64_t secs = (uint64_t)difftime(now,then);
	return seconds((uint64_t)secs);
}

bool GPSFix::hasEstimate(){
	return (latitude != 0 && longitude != 0) || (quality == 6);
}

bool GPSFix::setlock(bool locked){
	if (haslock != locked){
		haslock = locked;
		return true;
	}
	return false;
}

bool GPSFix::locked(){
	return haslock;
}

// Returns meters
double GPSFix::horizontalAccuracy(){
	if(longitudeDeviation != 0 && latitudeDeviation != 0) {		// If we have GxGST data
		return (sqrt(pow(longitudeDeviation, 2) + pow(latitudeDeviation, 2)));
	} else if (quality == 4) {									// If we have RTK Fix
		return 0.02 * horizontalDilution;						// 2x RTK accuracy from RTK1010 datasheet for 95% confidence
	}
	return 5.0 * horizontalDilution;							// 2x GPS accuracy from RTK1010 datasheet for 95% confidence
}

// Returns meters
double GPSFix::verticalAccuracy(){
	if(altitudeDeviation != 0) {								// If we have GxGST data
		return altitudeDeviation;
	} else if (quality == 4) {									// If we have RTK Fix
		return 0.02 * verticalDilution;							// 2x RTK accuracy from RTK1010 datasheet for 95% confidence
	}
	return 5.0 * verticalDilution;								// 2x GPS accuracy from RTK1010 datasheet for 95% confidence
}

// Takes a degree travel heading (0-360') and returns the name
std::string GPSFix::travelAngleToCompassDirection(double deg, bool abbrev){

	//normalize, just in case
	int32_t c = (int32_t)round(deg / 360.0 * 8.0);
	int32_t r = c % 8;
	if (r < 0){
		r = 8 + r;
	}

	if (abbrev){
		std::string dirs[] = {
			"N",
			"NE",
			"E",
			"SE",
			"S",
			"SW",
			"W",
			"NW",
			"N"
		};
		return dirs[r];
	}
	else {
		std::string dirs[] = {
			"North",
			"North East",
			"East",
			"South East",
			"South",
			"South West",
			"West",
			"North West",
			"North"
		};
		return dirs[r];
	}
	
};

double GPSFix::gsvAverageSNR(){

	double avg = 0;
	double relevant = 0;
	for (const auto& almanac : almanacTable) {
		for (const auto& satellite : almanac.second.satellites){
			if (satellite.snr > 0){
				relevant += 1.0;
			}
		}
	}

	for (const auto& almanac : almanacTable) {
		for (const auto& satellite : almanac.second.satellites){
			if (satellite.snr > 0){
				avg += satellite.snr;
			}
		}
	}
	avg /= relevant;

	return avg;
}
double GPSFix::gsvMinSNR(){
	double min = 9999999;
	if (almanacTable.empty()){
		return 0;
	}
	int32_t num_over_zero = 0;
	for (const auto& almanac : almanacTable) {
		for (const auto& satellite : almanac.second.satellites){
			if (satellite.snr > 0){
				num_over_zero++;
				if (satellite.snr < min){
					min = satellite.snr;
				}
			}
		}
	}
	if (num_over_zero == 0){
		return 0;
	}
	return min;
}

double GPSFix::gsvMaxSNR(){
	double max = 0;
	for (const auto& almanac : almanacTable) {
		for (const auto& satellite : almanac.second.satellites){
			if (satellite.snr > 0){
				if (satellite.snr > max){
					max = satellite.snr;
				}
			}
		}
	}
	return max;
}

uint32_t GPSFix::gsvVisibleSatellites() {
	uint32_t visibleSatelitesNum = 0;
	for (const auto& almanac : almanacTable) {
		visibleSatelitesNum += almanac.second.visibleSatelites;
	}
	return visibleSatelitesNum;
}

double GPSFix::gsvAlmanacPercentComplete() {
	if(almanacTable.empty()){
		return 0;
	}
	double percentComplete = 0;
	for (auto& almanac : almanacTable) {
		percentComplete += almanac.second.percentComplete();
	}	
	return percentComplete/almanacTable.size();
}

// ===========================================================
// ===================== PUBX03 QUERY METHODS =================
// ===========================================================

uint32_t GPSFix::pubxCountTracked() const {
	return (uint32_t)pubx03Almanac.satellites.size();
}

uint32_t GPSFix::pubxCountUsedInFix() const {
	uint32_t count = 0;
	for (const auto& sat : pubx03Almanac.satellites) {
		if (sat.usedInFix) count++;
	}
	return count;
}

uint32_t GPSFix::pubxCountUsedBySvid(uint32_t svid) const {
	for (const auto& sat : pubx03Almanac.satellites) {
		if (sat.svid == svid && sat.usedInFix) return 1;
	}
	return 0;
}

uint32_t GPSFix::pubxCountUsedByConstellation(uint8_t gnssId) const {
	uint32_t count = 0;
	for (const auto& sat : pubx03Almanac.satellites) {
		if (!sat.usedInFix) continue;
		if (gnssId == 0 && sat.svid >= 1 && sat.svid <= 32) count++;           // GPS
		else if (gnssId == 1 && sat.svid >= 120 && sat.svid <= 158) count++;    // SBAS
		else if (gnssId == 2 && sat.svid >= 211 && sat.svid <= 246) count++;    // Galileo
		else if (gnssId == 3 && (sat.svid >= 33 && sat.svid <= 90 || sat.svid >= 159 && sat.svid <= 163)) count++; // BeiDou
		else if (gnssId == 5 && sat.svid >= 193 && sat.svid <= 202) count++;    // QZSS
		else if (gnssId == 6 && sat.svid >= 65 && sat.svid <= 95) count++;      // GLONASS
		else if (gnssId == 7 && sat.svid >= 247 && sat.svid <= 253) count++;    // NavIC
	}
	return count;
}

double GPSFix::pubxAverageSNRUsed() const {
	double sum = 0;
	int count = 0;
	for (const auto& sat : pubx03Almanac.satellites) {
		if (sat.usedInFix && sat.cno > 0) {
			sum += sat.cno;
			count++;
		}
	}
	return count > 0 ? sum / count : 0;
}

double GPSFix::pubxAverageSNRAll() const {
	double sum = 0;
	int count = 0;
	for (const auto& sat : pubx03Almanac.satellites) {
		if (sat.cno > 0) {
			sum += sat.cno;
			count++;
		}
	}
	return count > 0 ? sum / count : 0;
}


std::string fixStatusToString(char status){
	switch (status){
	case 'A':
		return "Active";
	case 'V':
		return "Void";
	default:
		return "Unknown";
	}
}
std::string fixTypeToString(uint8_t type){
	switch (type){
	case 1:
		return "None";
	case 2:
		return "2D";
	case 3:
		return "3D";
	default:
		return "Unknown";
	}
}
std::string fixQualityToString(uint8_t quality){
	switch (quality){
	case 0:
		return "Invalid";
	case 1:
		return "Standard";
	case 2:
		return "DGPS";
	case 3:
		return "PPS Fix";
	case 4:
		return "RTK Fix";
	case 5:
		return "RTK Float";
	case 6:
		return "Estimate";
	default:
		return "Unknown";
	}
}

std::string GPSFix::toString(){
	stringstream ss;
	ios_base::fmtflags oldflags = ss.flags();

	ss << "========================== GPS FIX ================================" << endl
		<< " Status: \t\t" << ((haslock) ? "LOCK!" : "SEARCHING...") << endl
		<< " Satellites: \t\t" << trackingSatellites << " (tracking) of " << gsvVisibleSatellites() << " (visible)" << endl
		<< " < Fix Details >" << endl
		<< "   Age:                " << timeSince(last_epoch).count() << " s" << endl
		<< "   Timestamp:          " << last_epoch.toString() << "   UTC   \n\t\t\t(raw: " << last_epoch.rawTime << " time, " << last_epoch.rawDate << " date)" << endl
		<< "   Raw Status:         " << status			<< "  (" << fixStatusToString(status) << ")" << endl
		<< "   Type:               " << (int)type		<< "  (" << fixTypeToString(type) << ")" << endl
		<< "   Quality:            " << (int)quality	<< "  (" << fixQualityToString(quality) << ")" << endl
		<< "   Lat/Lon (N,E):      " << setprecision(6) << fixed << latitude << "' N, " << longitude << "' E" <<  endl;

	ss.flags(oldflags);  //reset
	ss << "   Diff (age, id):     " << diffAge << " s, " << diffStation << endl;

	ss << "   DOP (P,H,V):        " << dilution << ",   " << horizontalDilution << ",   " << verticalDilution << endl
		<< "   Error(lat,lon,alt): " << latitudeDeviation << " m,  " << longitudeDeviation << " m,  " << altitudeDeviation << " m" << endl
		<< "   Accuracy(H,V):      " << horizontalAccuracy() << " m,   " << verticalAccuracy() << " m" << endl;

	ss << "   Altitude:           " << altitude << " m" << endl
		<< "   Speed:              " << speed << " km/h" << endl
		<< "   Travel Dir:         " << travelAngle << " deg  [" << travelAngleToCompassDirection(travelAngle) << "]" << endl
		<< "   GSV SNR:             avg: " << gsvAverageSNR() << " dB   [min: " << gsvMinSNR() << " dB,  max:" << gsvMaxSNR() << " dB]" << endl;

	ss << " < Almanac (" << gsvAlmanacPercentComplete() << "%) >" << endl;
	if (almanacTable.empty()){
		ss << " > No satellite info in almanac." << endl;
	}
	for (auto& almanac : almanacTable) {
		ss << "   [" << almanac.first << "] Age: " << timeSince(almanac.second.lastUpdate).count() << " s" << endl;
		for (size_t i = 0; i < almanac.second.satellites.size(); i++) {
			ss << "      [" << setw(2) << setfill(' ') <<  (i + 1) << "]   " << almanac.second.satellites[i].toString() << endl;
		}
	}


	return ss.str();
}
GPSFix::operator std::string(){
	return toString();
}



