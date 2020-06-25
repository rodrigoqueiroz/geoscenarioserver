#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"
 
DECLARE_LOG_CATEGORY_EXTERN(GeoScenarioModule, All, All);
 
class FGeoScenarioModule : public IModuleInterface
{
	public:
		/** IModuleInterface implementation */
		virtual void StartupModule() override;
		virtual void ShutdownModule() override;
};