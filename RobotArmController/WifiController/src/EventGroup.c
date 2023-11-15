#include <WifiController/EventGroup.h>

static StaticEventGroup_t egBuffer;
static EventGroupHandle_t egHandle;

WifiController_ErrorCode_t WifiController_EventGroup_Init()
{
    egHandle = xEventGroupCreateStatic(&egBuffer);
    if (egHandle == NULL)
    {
        // The static constructor should only fail if the buffer is NULL. This should never happen.
        return WifiController_ErrorCode_UNKNOWN;
    }
    return WifiController_ErrorCode_NONE;
}

void WifiController_EventGroup_Delete()
{
    vEventGroupDelete(egHandle);
}

EventGroupHandle_t WifiController_EventGroup_GetHandle()
{
    return egHandle;
}
