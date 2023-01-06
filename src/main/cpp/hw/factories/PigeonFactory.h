
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

#pragma once

#include <hw/DragonPigeon.h>

/// @class PigeonFactory
/// @brief This controls the creation of analog inputs
class PigeonFactory
{
	public:


		/// @brief  Find or create the analog input factory
		/// @return PigeonFactory* pointer to the factory
		static PigeonFactory* GetFactory();

        DragonPigeon* GetPigeon
		(
			DragonPigeon::PIGEON_USAGE usage 
		) const;

		DragonPigeon* GetCenterPigeon() const { return m_centerPigeon; };
		DragonPigeon* GetShooterPigeon() const { return m_shooterPigeon; };

        DragonPigeon* CreatePigeon
		( 
			int canID, 
			std::string canBusName,
			DragonPigeon::PIGEON_TYPE type, 
			DragonPigeon::PIGEON_USAGE usage, 
			double rotation 
		);

	private:
		PigeonFactory();
		virtual ~PigeonFactory() = default;

        DragonPigeon* m_centerPigeon;
		DragonPigeon* m_shooterPigeon;

		static PigeonFactory*	m_factory;
};