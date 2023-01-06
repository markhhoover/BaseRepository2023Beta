#include <string>
#include <utils/Logger.h>
#include <pugixml/pugixml.hpp>
#include <utils/HardwareIDValidation.h>
#include <hw/xml/ThroughBoreEncoderXmlParser.h>
#include <frc/Encoder.h>

using namespace pugi;
using namespace std;
using namespace frc;

Encoder* ThroughBoreEncoderXmlParser::ParseXML
(
    string              networkTableName,
    pugi::xml_node      throughBoreEncoderNode
)
{
    string usage;
    int DIOA = 0;
    int DIOB = 0;
    int PWMID = 0;
    bool hasError = false;
    Encoder* throughboreencoder = nullptr;
    for ( xml_attribute attr = throughBoreEncoderNode.first_attribute(); attr && !hasError; attr = attr.next_attribute())
    {
        if ( strcmp (attr.name(), "usage") == 0 )
        {
            usage = attr.value();
        }
        else if (strcmp (attr.name(), "DIOA") == 0 )
        {
            DIOA = attr.as_int();
            hasError = HardwareIDValidation::ValidateDIOID( DIOA, string( "ThroughBoreEncoderXmlParser::ParseXML" ) );
        }
        else if ( strcmp( attr.name(), "DIOB ") == 0)
        {
            DIOB = attr.as_int();
            hasError = HardwareIDValidation::ValidateDIOID( DIOB, string( "ThroughBoreEncoderXmlParser::ParseXML" ) );
        }
        else if ( strcmp (attr.name(), "PWMID") == 0)
        {
            int iVal = attr.as_int();
            if (iVal >= 0 && iVal <= 9) //filler values todo need to find actual values or have ID Validation
            {
                PWMID = attr.as_int();
            }
            else
            {
                Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR_ONCE, string("ThroughBoreEncoderXmlParser::ParseXML"),string("invalid PWN ID \n"), string("iVal"));
                hasError = true;
            }
        }
    }
    if (!hasError)
    {
        throughboreencoder = new Encoder (/*usage,*/ DIOA, DIOB, PWMID ); //TODO add usage to dragon through bore encoder
    }
    return throughboreencoder;
}