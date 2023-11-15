#include "WifiController/ControlTable.h"

#include <FreeRTOS.h>

#include <string.h>

bool WifiController_ControlTable_Init(WifiController_ControlTable_t *this)
{
    this->head = this->tail = NULL;
    this->size = 0;
    return true;
}

void WifiController_ControlTable_Delete(WifiController_ControlTable_t *this)
{
    WifiController_ControlTable_Clear(this);
}

bool WifiController_ControlTable_Add(WifiController_ControlTable_t *this, const char *label, const char *control)
{
    WifiController_ControlTableListElement_t *newElement = (WifiController_ControlTableListElement_t*)pvPortMalloc(sizeof(WifiController_ControlTableListElement_t));
    if (newElement == NULL)
    {
        return false;
    }

    // Initialize the new ControlTableRow
    newElement->row.label = (char*)pvPortMalloc(strlen(label) + 1);
    newElement->row.control = (char*)pvPortMalloc(strlen(control) + 1);
    if (newElement->row.label == NULL || newElement->row.control == NULL)
    {
        vPortFree(newElement);
        return false;
    }
    strncpy(newElement->row.label, label, strlen(label) + 1);
    strncpy(newElement->row.control, control, strlen(control) + 1);
    newElement->next = NULL;

    if (this->tail == NULL)
    {
        // List is empty, set both head and tail to the new element
        this->head = this->tail = newElement;
    }
    else
    {
        // Add the new element to the end of the list
        this->tail->next = newElement;
        this->tail = newElement;
    }

    this->size++;
    return true;
}

bool WifiController_ControlTable_Remove(WifiController_ControlTable_t *this, int index)
{
    if (index < 0 || index >= this->size)
    {
        return false; // Index out of bounds
    }

    WifiController_ControlTableListElement_t *current = this->head;
    WifiController_ControlTableListElement_t *previous = NULL;

    // Traverse the list to find the element at the specified index
    for (int i = 0; i < index; i++)
    {
        previous = current;
        current = current->next;
    }

    if (current == this->head)
    {
        // If removing the head element
        this->head = current->next;
    }
    else if (current == this->tail)
    {
        // If removing the tail element
        this->tail = previous;
        previous->next = NULL;
    }
    else
    {
        // Removing an element from the middle
        previous->next = current->next;
    }

    // Free the memory of the removed element
    vPortFree(current->row.label);
    vPortFree(current->row.control);
    vPortFree(current);

    this->size--;
    return true;
}

void WifiController_ControlTable_Clear(WifiController_ControlTable_t *this)
{
    while (this->head != NULL)
    {
        WifiController_ControlTableListElement_t *temp = this->head;
        this->head = this->head->next;
        vPortFree(temp->row.label);
        vPortFree(temp->row.control);
        vPortFree(temp);
    }

    this->tail = NULL;
    this->size = 0;
}

int WifiController_ControlTable_Size(const WifiController_ControlTable_t *this)
{
    return this->size;
}

WifiController_ControlTableRow_t* ControlTable_Get(const WifiController_ControlTable_t *this, int index)
{
    if (index < 0 || index >= this->size)
    {
        return NULL; // Index out of bounds
    }

    WifiController_ControlTableListElement_t *current = this->head;
    for (int i = 0; i < index; i++)
    {
        current = current->next;
    }

    return &(current->row);
}

bool WifiController_ControlTable_Set(WifiController_ControlTable_t *this, int index, const char *label, const char *control)
{
    WifiController_ControlTableRow_t *row = WifiController_ControlTable_Get(this, index);
    if (row == NULL)
    {
        return false; // Index out of bounds
    }

    // Free the previous label and control strings
    vPortFree(row->label);
    vPortFree(row->control);

    // Update with new values
    row->label = (char*)pvPortMalloc(strlen(label) + 1);
    row->control = (char*)pvPortMalloc(strlen(control) + 1);
    if (row->label == NULL || row->control == NULL)
    {
        return false;
    }
    strncpy(row->label, label, strlen(label) + 1);
    strncpy(row->control, control, strlen(control) + 1);

    return true;
}

int WifiController_ControlTable_FindByLabel(const WifiController_ControlTable_t *this, const char *label)
{
    WifiController_ControlTableListElement_t *current = this->head;
    for (int index = 0; current != NULL; index++)
    {
        if (strcmp(current->row.label, label) == 0)
        {
            return index;
        }
        current = current->next;
    }

    return -1; // Label not found
}
