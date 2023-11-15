#include "WifiController/TextField.h"

#include <FreeRTOS.h>

#include <string.h>

static WifiController_TextFieldListElement_t* create_text_field_list_element(const char *text);
static void delete_text_field_list_element(WifiController_TextFieldListElement_t *this);

void WifiController_TextField_Init(WifiController_TextField_t *this)
{
    this->head = NULL;
    this->tail = NULL;
    this->size = 0;
}

void WifiController_TextField_Delete(WifiController_TextField_t *this)
{
    WifiController_TextFieldListElement_t *current = this->head;
    while (current != NULL)
    {
        WifiController_TextFieldListElement_t *next = current->next;
        delete_text_field_list_element(current);
        vPortFree(current);
        current = next;
    }
    this->head = NULL;
    this->tail = NULL;
    this->size = 0;
}

int WifiController_TextField_GetSize(const WifiController_TextField_t *this)
{
    return this->size;
}

bool WifiController_TextField_PushBack(WifiController_TextField_t *this, const char *text)
{
    WifiController_TextFieldListElement_t *new_element = create_text_field_list_element(text);
    if (new_element == NULL)
    {
        return false;
    }

    // If the list is empty, set the new element as both root and tail
    if (this->head == NULL)
    {
        this->head = new_element;
        this->tail = new_element;
    }
    else
    {
        // Otherwise, update the tail to point to the new element
        this->tail->next = new_element;
        this->tail = new_element;
    }

    // Increment the size of the list
    this->size++;
    return true;
}

const char* WifiController_TextField_At(const WifiController_TextField_t *this, int index)
{
    if ((index < 0) || (index >= this->size))
    {
        return NULL; // Index out of bounds
    }

    WifiController_TextFieldListElement_t *current = this->head;
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

static void delete_text_field_list_element(WifiController_TextFieldListElement_t *this)
{
    vPortFree(this->text);
}
