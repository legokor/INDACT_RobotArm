#ifndef WIFICONTROLLER_ACTIONLIST_H_
#define WIFICONTROLLER_ACTIONLIST_H_

#include <FreeRTOS.h>
#include <semphr.h>

#include <stdbool.h>

/**
 * @brief Structure representing a single action.
 */
typedef struct WifiController_Action
{
    char *path; /**< The path that identifies the action. */
    void (*handler)(const char *); /**< The handler function that will be called when the action is requested. */
} WifiController_Action_t;

/**
 * @brief Structure representing a single element of the action list.
 */
typedef struct WifiController_ActionListElement
{
    WifiController_Action_t action; /**< The action. */
    struct WifiController_ActionListElement *next; /**< The next element in the list. */
} WifiController_ActionListElement_t;

/**
 * @brief Structure representing the action list.
 * 
 * @note The fields of this structure should not be accessed directly. Use the provided functions instead.
 */
typedef struct WifiController_ActionList
{
    WifiController_ActionListElement_t *head; /**< The head of the list. */
    WifiController_ActionListElement_t *tail; /**< The tail of the list. */
    int size; /**< The size of the list. */
    SemaphoreHandle_t mutex; /**< The mutex used to protect the list from concurrent access. */
} WifiController_ActionList_t;

/**
 * @brief Initializes the action list.
 * 
 * @param this The action list.
 * @return True if the action list was initialized successfully, false otherwise.
 * 
 * @note This function must be called before any other function of the action list.
 * @note The memory for the @p this struct must be allocated before calling the function.
 */
bool WifiController_ActionList_Init(WifiController_ActionList_t *this);

/**
 * @brief Deinitialize the action list.
 * 
 * @param this The action list.
 * 
 * @note This function must be called when the action list is no longer needed.
 */
void WifiController_ActionList_Delete(WifiController_ActionList_t *this);

/**
 * @brief Adds an action to the action list.
 * 
 * @param this The action list.
 * @param path The path that identifies the action.
 * @param handler The handler function that will be called when the action is requested.
 * @return True if the action was added successfully, false otherwise.
 */
bool WifiController_ActionList_Add(WifiController_ActionList_t *this, const char *path, void (*handler)(const char *));

/**
 * @brief Removes an action from the action list.
 * 
 * @param this The action list.
 * @param index The index of the action to remove.
 * @return True if the action was removed successfully, false otherwise.
 */
bool WifiController_ActionList_Remove(WifiController_ActionList_t *this, int index);

/**
 * @brief Removes all the actions from the action list.
 * 
 * @param this The action list.
 */
void WifiController_ActionList_Clear(WifiController_ActionList_t *this);

/**
 * @brief Gets the size of the action list.
 * 
 * @param this The action list.
 * @return The size of the action list.
 */
int WifiController_ActionList_GetSize(const WifiController_ActionList_t *this);

/**
 * @brief Gets the action at the specified index.
 * 
 * @param this The action list.
 * @param index The index of the action to get.
 * @return The action at the specified index.
 */
const WifiController_Action_t *WifiController_ActionList_At(const WifiController_ActionList_t *this, int index);

/**
 * @brief Finds the index of the action with the specified path.
 * 
 * @param this The action list.
 * @param path The path of the action to find.
 * @return The index of the action with the specified path, or -1 if the action was not found.
 */
int WifiController_ActionList_FindByPath(const WifiController_ActionList_t *this, const char *path);

#endif /* WIFICONTROLLER_ACTIONLIST_H_ */
