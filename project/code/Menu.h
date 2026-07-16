#ifndef CODE_MENU_H_
#define CODE_MENU_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Menu service.
 * Call it repeatedly from the main loop. Key flags are consumed internally.
 */
void Menu(void);

/*
 * Returns 1 only during the first callback invocation after entering an
 * action item. Use it when a callback contains one-shot initialization.
 */
int Menu_IsActionFirstCall(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_MENU_H_ */
