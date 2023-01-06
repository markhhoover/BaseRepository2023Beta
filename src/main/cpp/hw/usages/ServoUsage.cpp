
//====================================================================================================================================================
// Copyright 2022 Lake Orion Robotics FIRST Team 302 
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

// C++ Includes
#include <map>
#include <memory>
#include <string>

// FRC includes

// Team 302 includes
#include <hw/usages/ServoUsage.h>
#include <utils/Logger.h>

// Third Party Includes

using namespace std;

ServoUsage* ServoUsage::m_instance = nullptr;
ServoUsage* ServoUsage::GetInstance()
{
    if ( m_instance == nullptr )
    {
        m_instance = new ServoUsage();
    }
    return m_instance;
}

ServoUsage::ServoUsage()
{
    m_usageMap["RELEASE_SERVO"]  = SERVO_USAGE::RELEASE_SERVO;
    m_usageMap["RELEASE_SERVO2"]  = SERVO_USAGE::RELEASE_SERVO2;
    m_usageMap["FLAG_SERVO"]  = SERVO_USAGE::FLAG_SERVO;
}

ServoUsage::~ServoUsage()
{
    m_usageMap.clear();
}

ServoUsage::SERVO_USAGE ServoUsage::GetUsage
(
    const string              usageString
)
{
    auto it = m_usageMap.find(usageString);
    if (it != m_usageMap.end())
    {
        return it->second;
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("ServoUsage::GetUsage"), string("unknown usage"), usageString);
    return ServoUsage::SERVO_USAGE::UNKNOWN_SERVO_USAGE;
}

