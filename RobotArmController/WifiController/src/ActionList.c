#include "WifiController/ActionList.h"

#include <string.h>

static bool add(WifiController_ActionList_t *this, const char *path, void (*handler)(const char*));
static bool remove(WifiController_ActionList_t *this, int index);
static void clear(WifiController_ActionList_t *this);
static int get_size(const WifiController_ActionList_t *this);
static const WifiController_Action_t* at(const WifiController_ActionList_t *this, int index);
static int find_by_index(const WifiController_ActionList_t *this, const char *path);

bool WifiController_ActionList_Init(WifiController_ActionList_t *this)
{
    this->head = this->tail = NULL;
    this->size = 0;
    this->mutex = xSemaphoreCreateMutex();
    // Check if the mutex was created successfully
    if (this->mutex == NULL)
    {
        return false;
    }
    return true;
}

void WifiController_ActionList_Delete(WifiController_ActionList_t *this)
{
    WifiController_ActionList_Clear(this);
    vSemaphoreDelete(this->mutex);
}

bool WifiController_ActionList_Add(WifiController_ActionList_t *this, const char *path, void (*handler)(const char*))
{
    if (xSemaphoreTake(this->mutex, portMAX_DELAY) != pdTRUE)
    {
        return false;
    }
    bool ret_val = add(this, path, handler);
    xSemaphoreGive(this->mutex);
    return ret_val;
}

bool WifiController_ActionList_Remove(WifiController_ActionList_t *this, int index)
{
    if (xSemaphoreTake(this->mutex, portMAX_DELAY) != pdTRUE)
    {
        return false;
    }
    bool rv = remove(this, index);
    xSemaphoreGive(this->mutex);
    return rv;
}

void WifiController_ActionList_Clear(WifiController_ActionList_t *this)
{
    if (xSemaphoreTake(this->mutex, portMAX_DELAY) != pdTRUE)
    {
        return;
    }
    clear(this);
    xSemaphoreGive(this->mutex);
}

int WifiController_ActionList_GetSize(const WifiController_ActionList_t *this)
{
    if (xSemaphoreTake(this->mutex, portMAX_DELAY) != pdTRUE)
    {
        return -1;
    }
    int rv = get_size(this);
    xSemaphoreGive(this->mutex);
    return rv;
}

const WifiController_Action_t* WifiController_ActionList_At(const WifiController_ActionList_t *this, int index)
{
    if (xSemaphoreTake(this->mutex, portMAX_DELAY) != pdTRUE)
    {
        return NULL;
    }
    const WifiController_Action_t *rv = at(this, index);
    xSemaphoreGive(this->mutex);
    return rv;
}

int WifiController_ActionList_FindByPath(const WifiController_ActionList_t *this, const char *path)
{
    if (xSemaphoreTake(this->mutex, portMAX_DELAY) != pdTRUE)
    {
        return -1;
    }
    int rv = find_by_index(this, path);
    xSemaphoreGive(this->mutex);
    return rv;
}

static bool add(WifiController_ActionList_t *this, const char *path, void (*handler)(const char*))
{
    WifiController_ActionListElement_t *newElement = (WifiController_ActionListElement_t*)pvPortMalloc(sizeof(WifiController_ActionListElement_t));
    if (newElement == NULL)
    {
        // Memory allocation error
        return false;
    }

    newElement->action.path = (char*)pvPortMalloc(strlen(path) + 1);
    if (newElement->action.path == NULL)
    {
        // Memory allocation error
        vPortFree(newElement);
        return false;
    }
    strncpy(newElement->action.path, path, strlen(path) + 1);
    newElement->action.handler = handler;

    newElement->next = NULL;

    if (this->tail == NULL)
    {
        this->head = this->tail = newElement;
    }
    else
    {
        this->tail->next = newElement;
        this->tail = newElement;
    }

    this->size++;
    return true;
}

static bool remove(WifiController_ActionList_t *this, int index)
{
    if ((index < 0) || (index >= this->size))
    {
        return false;
    }

    WifiController_ActionListElement_t *current = this->head;
    WifiController_ActionListElement_t *previous = NULL;

    for (int i = 0; i < index; i++)
    {
        previous = current;
        current = current->next;
    }

    if (current == this->head)
    {
        this->head = current->next;
    }
    else if (current == this->tail)
    {
        this->tail = previous;
        previous->next = NULL;
    }
    else
    {
        previous->next = current->next;
    }

    vPortFree(current->action.path);
    vPortFree(current);

    this->size--;
    return true;
}

static void clear(WifiController_ActionList_t *this)
{
    while (this->head != NULL)
    {
        WifiController_ActionListElement_t *temp = this->head;
        this->head = this->head->next;
        vPortFree(temp->action.path);
        vPortFree(temp);
    }

    this->tail = NULL;
    this->size = 0;
}

static int get_size(const WifiController_ActionList_t *this)
{
    return this->size;
}

static const WifiController_Action_t* at(const WifiController_ActionList_t *this, int index)
{
    if (index < 0 || index >= this->size)
    {
        return NULL;
    }

    WifiController_ActionListElement_t *current = this->head;
    for (int i = 0; i < index; i++)
    {
        current = current->next;
    }

    return &(current->action);
}

static int find_by_index(const WifiController_ActionList_t *this, const char *path)
{
    int index = 0;
    for (WifiController_ActionListElement_t *current = this->head; current != NULL; current = current->next)
    {
        if (strcmp(current->action.path, path) == 0)
        {
            return index;
        }
        ++index;
    }
    return -1;
}
