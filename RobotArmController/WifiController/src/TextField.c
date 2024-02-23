#include "WifiController/TextField.h"

#include <FreeRTOS.h>

#include <string.h>

static WifiController_TextFieldListElement_t* create_text_field_list_element(const char *text);
static void delete_text_field_list_element(WifiController_TextFieldListElement_t *tfl);

void WifiController_TextField_Init(WifiController_TextFieldList_t *tfl)
{
    tfl->head = NULL;
    tfl->tail = NULL;
    tfl->size = 0;
}

void WifiController_TextField_Delete(WifiController_TextFieldList_t *tfl)
{
    WifiController_TextFieldListElement_t *current = tfl->head;
    while (current != NULL)
    {
        WifiController_TextFieldListElement_t *next = current->next;
        delete_text_field_list_element(current);
        vPortFree(current);
        current = next;
    }
    tfl->head = NULL;
    tfl->tail = NULL;
    tfl->size = 0;
}

int WifiController_TextField_GetSize(const WifiController_TextFieldList_t *tfl)
{
    return tfl->size;
}

bool WifiController_TextField_PushBack(WifiController_TextFieldList_t *tfl, const char *text)
{
    WifiController_TextFieldListElement_t *new_element = create_text_field_list_element(text);
    if (new_element == NULL)
    {
        return false;
    }

    // If the list is empty, set the new element as both root and tail
    if (tfl->head == NULL)
    {
        tfl->head = new_element;
        tfl->tail = new_element;
    }
    else
    {
        // Otherwise, update the tail to point to the new element
        tfl->tail->next = new_element;
        tfl->tail = new_element;
    }

    // Increment the size of the list
    tfl->size++;
    return true;
}

const char* WifiController_TextField_At(const WifiController_TextFieldList_t *tfl, int index)
{
    if ((index < 0) || (index >= tfl->size))
    {
        return NULL; // Index out of bounds
    }

    WifiController_TextFieldListElement_t *current = tfl->head;
    int i = 0;
    while (i < index)
    {
        current = current->next;
        i++;
    }

    return current->text;
}

static WifiController_TextFieldListElement_t* create_text_field_list_element(const char *text)
{
    WifiController_TextFieldListElement_t *new_element =
            (WifiController_TextFieldListElement_t*)pvPortMalloc(sizeof(WifiController_TextFieldListElement_t));
    if (new_element == NULL)
    {
        return NULL;
    }

    new_element->text = (char*)pvPortMalloc(strlen(text) + 1);
    if (new_element->text == NULL)
    {
        vPortFree(new_element);
        return NULL;
    }
    strcpy(new_element->text, text);

    new_element->next = NULL;

    return new_element;
}

static void delete_text_field_list_element(WifiController_TextFieldListElement_t *tfl)
{
    vPortFree(tfl->text);
}
