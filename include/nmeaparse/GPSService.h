/*
 * GPSService.h
 *
 *  Created on: Aug 14, 2014
 *      Author: Cameron Karlsson
 *
 *  See the license file included with this source.
 */

#ifndef GPSSERVICE_H_
#define GPSSERVICE_H_

#include <string>
#include <chrono>
#include <functional>
#include <nmeaparse/GPSFix.h>
#include <nmeaparse/NMEAParser.h>
#include <nmeaparse/Event.h>

namespace nmea {

const auto talkerIds = {"GA", "GB", "GL", "GN", "GP", "GQ"};

class GPSService {
private:

	void read_PSRF150(const NMEASentence& nmea);
	void read_GxGGA	(const NMEASentence& nmea);
	void read_GxGSA	(const NMEASentence& nmea);
	void read_GxGST	(const NMEASentence& nmea);
	void read_GxGSV	(const NMEASentence& nmea);
	void read_GxRMC	(const NMEASentence& nmea);
	void read_GxVTG	(const NMEASentence& nmea);

public:
	GPSFix fix;

	GPSService(NMEAParser& parser);
	virtual ~GPSService();

	Event<void(bool)> onLockStateChanged;		// user assignable handler, called whenever lock changes
	Event<void()> onUpdate;						// user assignable handler, called whenever fix changes

	void attachToParser(NMEAParser& parser);			// will attach to this parser's nmea sentence events
};


}

#endif /* GPSSERVICE_H_ */
