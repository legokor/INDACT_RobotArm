#include "WifiController/UserInterface.h"

#include <FreeRTOS.h>

#include <string.h>

#include "cJSON/cJSON.h"

bool WifiController_UserInterface_Init(WifiController_UserInterface_t *this)
{
    this->title = NULL;
    this->pageHeader = NULL;
    WifiController_TextField_Init(&this->textFieldTop);
    WifiController_TextField_Init(&this->textFieldBottom);
    WifiController_ControlTable_Init(&this->controlTable);
    this->userScript = NULL;
    return true;
}

void WifiController_UserInterface_Delete(WifiController_UserInterface_t *this)
{
    vPortFree(this->title);
    vPortFree(this->pageHeader);
    WifiController_TextField_Delete(&this->textFieldTop);
    WifiController_TextField_Delete(&this->textFieldBottom);
    WifiController_ControlTable_Delete(&this->controlTable);
    vPortFree(this->userScript);
}

bool WifiController_UserInterface_SetTitle(WifiController_UserInterface_t *this, const char *value)
{
    vPortFree(this->title);
    this->title = (char*)pvPortMalloc(sizeof(char) * (strlen(value) + 1));
    if (this->title == NULL)
    {
        return false;
    }
    strcpy(this->title, value);
    return true;
}

bool WifiController_UserInterface_SetPageHeader(WifiController_UserInterface_t *this, const char *value)
{
    vPortFree(this->pageHeader);
    this->pageHeader = (char*)pvPortMalloc(sizeof(char) * (strlen(value) + 1));
    if (this->pageHeader == NULL)
    {
        return false;
    }
    strcpy(this->pageHeader, value);
    return true;
}

bool WifiController_UserInterface_SetUserScript(WifiController_UserInterface_t *this, const char *value)
{
    vPortFree(this->userScript);
    this->userScript = (char*)pvPortMalloc(sizeof(char) * (strlen(value) + 1));
    if (this->userScript == NULL)
    {
        return false;
    }
    strcpy(this->userScript, value);
    return true;
}

int WifiController_UserInterface_GetConfigurationJSON(const WifiController_UserInterface_t *this, char *buffer, int max_length)
{
    // I leave this code here for reference in case using cJSON is not an option.

    // // TODO: Do something about max_length.
    // buffer[0] = '\0';
    // if (!check_value_init(this))
    // {
    //     return -1;
    // }

    // strcat(buffer, "{");
    // // title
    // strcat(buffer, "\"title\":\"");
    // strcat(buffer, this->title);
    // strcat(buffer, "\"");
    // // pageHeader
    // strcat(buffer, ",\"pageHeader\":\"");
    // strcat(buffer, this->pageHeader);
    // strcat(buffer, "\"");
    // // textFieldTop
    // strcat(buffer, ",\"textFieldTop\":[");
    // for (int i = 0; i < WifiController_TextField_GetSize(this->textFieldTop); ++i)
    // {
    //     if (i != 0)
    //     {
    //         strcat(buffer, ",");
    //     }
    //     strcat(buffer, "\"");
    //     strcat(buffer, WifiController_TextField_At(this->textFieldTop, i));
    //     strcat(buffer, "\"");
    // }
    // strcat(buffer, "]");
    // // textFieldBottom
    // strcat(buffer, ",\"textFieldBottom\":[");
    // for (int i = 0; i < WifiController_TextField_GetSize(this->textFieldBottom); ++i)
    // {
    //     if (i != 0)
    //     {
    //         strcat(buffer, ",");
    //     }
    //     strcat(buffer, "\"");
    //     strcat(buffer, WifiController_TextField_At(this->textFieldBottom, i));
    //     strcat(buffer, "\"");
    // }
    // strcat(buffer, "]");
    // // controlTable
    // strcat(buffer, ",\"controlTable\":[");
    // for (int i = 0; i < WifiController_ControlTable_Size(this->controlTable); ++i)
    // {
    //     if (i != 0)
    //     {
    //         strcat(buffer, ",");
    //     }
    //     strcat(buffer, "{\"label\":\"");
    //     strcat(buffer, WifiController_ControlTable_Get(this->controlTable, i)->label);
    //     strcat(buffer, "\",\"control\":\"");
    //     strcat(buffer, WifiController_ControlTable_Get(this->controlTable, i)->control);
    //     strcat(buffer, "\"}");
    // }
    // strcat(buffer, "]");
    // // userScript
    // strcat(buffer, ",\"userScript\":\"");
    // strcat(buffer, this->userScript);
    // strcat(buffer, "\"");
    // strcat(buffer, "}");

    // return strlen(buffer);

    cJSON *root = cJSON_CreateObject();
    cJSON *textFieldTop = cJSON_CreateArray();
    cJSON *textFieldBottom = cJSON_CreateArray();
    cJSON *controlTable = cJSON_CreateArray();

    cJSON_AddItemToObject(root, "title", cJSON_CreateString(this->title));
    cJSON_AddItemToObject(root, "pageHeader", cJSON_CreateString(this->pageHeader));

    for (int i = 0; i < WifiController_TextField_GetSize(&this->textFieldTop); ++i)
    {
        cJSON_AddItemToArray(textFieldTop, cJSON_CreateString(WifiController_TextField_At(&this->textFieldTop, i)));
    }
    cJSON_AddItemToObject(root, "textFieldTop", textFieldTop);

    for (int i = 0; i < WifiController_TextField_GetSize(&this->textFieldBottom); ++i)
    {
        cJSON_AddItemToArray(textFieldBottom, cJSON_CreateString(WifiController_TextField_At(&this->textFieldBottom, i)));
    }
    cJSON_AddItemToObject(root, "textFieldBottom", textFieldBottom);

    for (int i = 0; i < WifiController_ControlTable_GetSize(&this->controlTable); ++i)
    {
        cJSON *control = cJSON_CreateObject();
        cJSON_AddItemToObject(control, "label", cJSON_CreateString(WifiController_ControlTable_At(&this->controlTable, i)->label));
        cJSON_AddItemToObject(control, "control", cJSON_CreateString(WifiController_ControlTable_At(&this->controlTable, i)->control));
        cJSON_AddItemToArray(controlTable, control);
    }
    cJSON_AddItemToObject(root, "controlTable", controlTable);

    cJSON_AddItemToObject(root, "userScript", cJSON_CreateString(this->userScript));

    int length = -1;
    // The buffer needs to be 5 bytes bigger than the max_length. This is required by
    // cJSON_PrintPreallocated.
    if (cJSON_PrintPreallocated(root, buffer, max_length - 5, 0) == cJSON_True)
    {
        length = strlen(buffer);
    }

    cJSON_Delete(root);
    return length;
}

int WifiController_UserInterface_GetDataJSON(const WifiController_UserInterface_t *this, char *buffer, int max_length)
{
    // I leave this code here for reference in case using cJSON is not an option.

    // // TODO: Do something about max_length.
    // buffer[0] = '\0';
    // if (!check_value_init(this))
    // {
    //     return -1;
    // }

    // strcat(buffer, "{");
    // // textFieldTop
    // strcat(buffer, "\"textFieldTop\":[");
    // for (int i = 0; i < WifiController_TextField_GetSize(this->textFieldTop); ++i)
    // {
    //     if (i != 0)
    //     {
    //         strcat(buffer, ",");
    //     }
    //     strcat(buffer, "\"");
    //     strcat(buffer, WifiController_TextField_At(this->textFieldTop, i));
    //     strcat(buffer, "\"");
    // }
    // strcat(buffer, "]");
    // // textFieldBottom
    // strcat(buffer, ",\"textFieldBottom\":[");
    // for (int i = 0; i < WifiController_TextField_GetSize(this->textFieldBottom); ++i)
    // {
    //     if (i != 0)
    //     {
    //         strcat(buffer, ",");
    //     }
    //     strcat(buffer, "\"");
    //     strcat(buffer, WifiController_TextField_At(this->textFieldBottom, i));
    //     strcat(buffer, "\"");
    // }
    // strcat(buffer, "]");
    // // controlTable
    // strcat(buffer, ",\"controlTable\":[");
    // for (int i = 0; i < WifiController_ControlTable_Size(this->controlTable); ++i)
    // {
    //     if (i != 0)
    //     {
    //         strcat(buffer, ",");
    //     }
    //     strcat(buffer, "{\"label\":\"");
    //     strcat(buffer, WifiController_ControlTable_Get(this->controlTable, i)->label);
    //     strcat(buffer, "\",\"control\":\"");
    //     strcat(buffer, WifiController_ControlTable_Get(this->controlTable, i)->control);
    //     strcat(buffer, "\"}");
    // }
    // strcat(buffer, "]");
    // strcat(buffer, "}");

    // return strlen(buffer);

    cJSON *root = cJSON_CreateObject();
    cJSON *textFieldTop = cJSON_CreateArray();
    cJSON *textFieldBottom = cJSON_CreateArray();
    cJSON *controlTable = cJSON_CreateArray();

    for (int i = 0; i < WifiController_TextField_GetSize(&this->textFieldTop); ++i)
    {
        cJSON_AddItemToArray(textFieldTop, cJSON_CreateString(WifiController_TextField_At(&this->textFieldTop, i)));
    }
    cJSON_AddItemToObject(root, "textFieldTop", textFieldTop);

    for (int i = 0; i < WifiController_TextField_GetSize(&this->textFieldBottom); ++i)
    {
        cJSON_AddItemToArray(textFieldBottom, cJSON_CreateString(WifiController_TextField_At(&this->textFieldBottom, i)));
    }
    cJSON_AddItemToObject(root, "textFieldBottom", textFieldBottom);

    for (int i = 0; i < WifiController_ControlTable_GetSize(&this->controlTable); ++i)
    {
        cJSON *control = cJSON_CreateObject();
        cJSON_AddItemToObject(control, "label", cJSON_CreateString(WifiController_ControlTable_At(&this->controlTable, i)->label));
        cJSON_AddItemToObject(control, "control", cJSON_CreateString(WifiController_ControlTable_At(&this->controlTable, i)->control));
        cJSON_AddItemToArray(controlTable, control);
    }
    cJSON_AddItemToObject(root, "controlTable", controlTable);

    int length = -1;
    // The buffer needs to be 5 bytes bigger than the max_length. This is required by
    // cJSON_PrintPreallocated.
    if (cJSON_PrintPreallocated(root, buffer, max_length - 5, 0) == cJSON_True)
    {
        length = strlen(buffer);
    }

    cJSON_Delete(root);
    return length;
}
