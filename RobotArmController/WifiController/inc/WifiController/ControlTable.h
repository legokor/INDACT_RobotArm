#ifndef WIFICONTROLLER_CONTROLTABLE_H_
#define WIFICONTROLLER_CONTROLTABLE_H_

#include <stdbool.h>

typedef struct WifiController_ControlTableRow
{
    char *label;
    char *control;
} WifiController_ControlTableRow_t;

typedef struct WifiController_ControlTableListElement
{
    WifiController_ControlTableRow_t row;
    struct WifiController_ControlTableListElement *next;
} WifiController_ControlTableListElement_t;

typedef struct WifiController_ControlTableList
{
    WifiController_ControlTableListElement_t *head;
    WifiController_ControlTableListElement_t *tail;
    int size;
} WifiController_ControlTableList_t;

typedef WifiController_ControlTableList_t WifiController_ControlTable_t;

void WifiController_ControlTable_Init(WifiController_ControlTable_t *this);

// Function to release the resources held by the list
void WifiController_ControlTable_Delete(WifiController_ControlTable_t *this);

// Function to add a new ControlTableRow to the end of the list
bool WifiController_ControlTable_Add(WifiController_ControlTable_t *this, const char *label, const char *control);

// Function to remove a ControlTableRow at a specific index
bool WifiController_ControlTable_Remove(WifiController_ControlTable_t *this, int index);

// Function to clear the entire list
void WifiController_ControlTable_Clear(WifiController_ControlTable_t *this);

// Function to retrieve the number of elements in the list
int WifiController_ControlTable_GetSize(const WifiController_ControlTable_t *this);

// Function to get a ControlTableRow at a specific index
WifiController_ControlTableRow_t* WifiController_ControlTable_At(const WifiController_ControlTable_t *this, int index);

// Function to modify a ControlTableRow at a specific index
bool WifiController_ControlTable_Set(WifiController_ControlTable_t *this, int index, const char *label, const char *control);

// Function to search for a ControlTableRow with a specific label
int WifiController_ControlTable_FindByLabel(const WifiController_ControlTable_t *this, const char *label);

#endif /* WIFICONTROLLER_CONTROLTABLE_H_ */
