/*
 * Copyright (C) 2008 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*-------------------------------------------------------------------*/
#include "includes.h"
#include "scanmerge.h"
#include "shlist.h"

#define IS_HIDDEN_AP(a)	(((a)->ssid_len == 0) || ((a)->ssid[0] == '\0'))
/*-----------------------------------------------------------------------------
Routine Name: scan_init
Routine Description: Inits scan merge list
Arguments:
   mydrv   - pointer to private driver data structure
Return Value:
-----------------------------------------------------------------------------*/
void scan_init( struct wpa_driver_ti_data *mydrv )
{
    mydrv->last_scan = -1;
    shListInitList( &(mydrv->scan_merge_list) );
}

/*-----------------------------------------------------------------------------
Routine Name: scan_free
Routine Description: Frees scan structure private data
Arguments:
   ptr - pointer to private data structure
Return Value:
-----------------------------------------------------------------------------*/
static void scan_free( void *ptr )
{
    os_free( ptr );
}

/*-----------------------------------------------------------------------------
Routine Name: scan_exit
Routine Description: Cleans scan merge list
Arguments:
   mydrv   - pointer to private driver data structure
Return Value:
-----------------------------------------------------------------------------*/
void scan_exit( struct wpa_driver_ti_data *mydrv )
{
    shListDelAllItems( &(mydrv->scan_merge_list), scan_free );
}

/*-----------------------------------------------------------------------------
Routine Name: scan_equal
Routine Description: Compares bssid of scan result and scan merge structure
Arguments:
   val   - pointer to scan result structure
   idata - pointer to scan merge structure
Return Value: 1 - if equal, 0 - if not
-----------------------------------------------------------------------------*/
static int scan_equal( void *val,  void *idata )
{
    struct wpa_scan_result *new_res = (struct wpa_scan_result *)val;
    struct wpa_scan_result *lst_res =
               (struct wpa_scan_result *)(&(((scan_merge_t *)idata)->scanres));
    int ret;
    size_t len;

    len = (IS_HIDDEN_AP(new_res) || IS_HIDDEN_AP(lst_res)) ?
          0 : new_res->ssid_len;
    ret = ((lst_res->ssid_len != new_res->ssid_len) && (len != 0)) ||
          (os_memcmp(new_res->bssid, lst_res->bssid, ETH_ALEN) ||
           os_memcmp(new_res->ssid, lst_res->ssid, len));
    return !ret;
}

/*-----------------------------------------------------------------------------
Routine Name: copy_scan_res
Routine Description: copies scan result structure to scan merge list item
Arguments:
   dst - pointer to scan result structure in the list
   src - source pointer to scan result structure
Return Value: NONE
-----------------------------------------------------------------------------*/
void copy_scan_res( struct wpa_scan_result *dst, struct wpa_scan_result *src )
{
    if( IS_HIDDEN_AP(src) ) {
        os_memcpy( src->ssid, dst->ssid, dst->ssid_len );
        src->ssid_len = dst->ssid_len;
    }
    os_memcpy( dst, src, sizeof(struct wpa_scan_result) );
}

/*-----------------------------------------------------------------------------
Routine Name: scan_add
Routine Description: adds scan result structure to scan merge list
Arguments:
   head    - pointer to scan merge list head
   res_ptr - pointer to scan result structure
Return Value: Pointer to scan merge item
-----------------------------------------------------------------------------*/
static scan_merge_t *scan_add( SHLIST *head, struct wpa_scan_result *res_ptr )
{
    scan_merge_t *scan_ptr;

    scan_ptr = (scan_merge_t *)os_malloc( sizeof(scan_merge_t) );
    if( !scan_ptr )
        return( NULL );
    os_memcpy( &(scan_ptr->scanres), res_ptr, sizeof(struct wpa_scan_result) );
    scan_ptr->count = SCAN_MERGE_COUNT;
    shListInsLastItem( head, (void *)scan_ptr );
    return scan_ptr;
}

/*-----------------------------------------------------------------------------
Routine Name: scan_find
Routine Description: Looks for scan merge item in scan results array
Arguments:
   scan_ptr - pointer to scan merge item
   results - pointer to scan results array
   number_items - current number of items
Return Value: 1 - if item was found, 0 - otherwise
-----------------------------------------------------------------------------*/
static int scan_find( scan_merge_t *scan_ptr, struct wpa_scan_result *results,
                      unsigned int number_items )
{
    unsigned int i;

    for(i=0;( i < number_items );i++) {
        if( scan_equal( &(results[i]), scan_ptr ) )
            return 1;
    }
    return 0;
}

/*-----------------------------------------------------------------------------
Routine Name: scan_merge
Routine Description: Merges current scan results with previous
Arguments:
   mydrv   - pointer to private driver data structure
   results - pointer to scan results array
   number_items - current number of items
   max_size - maximum namber of items
Return Value: Merged number of items
-----------------------------------------------------------------------------*/
unsigned int scan_merge( struct wpa_driver_ti_data *mydrv,
                         struct wpa_scan_result *results, int force_flag,
                         unsigned int number_items, unsigned int max_size )
{
    SHLIST *head = &(mydrv->scan_merge_list);
    SHLIST *item, *del_item;
    scan_merge_t *scan_ptr;
    unsigned int i;

    /* Prepare items for removal */
    item = shListGetFirstItem( head );
    while( item != NULL ) {
        scan_ptr = (scan_merge_t *)(item->data);
        if( scan_ptr->count != 0 )
            scan_ptr->count--;
        item = shListGetNextItem( head, item );
    }

    for(i=0;( i < number_items );i++) { /* Find/Add new items */
        item = shListFindItem( head, &(results[i]), scan_equal );
        if( item ) {
            scan_ptr = (scan_merge_t *)(item->data);
            copy_scan_res(&(scan_ptr->scanres), &(results[i]));
            scan_ptr->count = SCAN_MERGE_COUNT;
        }
        else {
            scan_add( head, &(results[i]) );
        }
    }

    item = shListGetFirstItem( head );  /* Add/Remove missing items */
    while( item != NULL ) {
        del_item = NULL;
        scan_ptr = (scan_merge_t *)(item->data);
        if( scan_ptr->count != SCAN_MERGE_COUNT ) {
            if( !force_flag && ((scan_ptr->count == 0) ||
                (mydrv->last_scan == SCAN_TYPE_NORMAL_ACTIVE)) )
                del_item = item;
            else {
                if( number_items < max_size ) {
                    os_memcpy(&(results[number_items]),
                          &(scan_ptr->scanres),sizeof(struct wpa_scan_result));
                    number_items++;
                }
            }
        }
        item = shListGetNextItem( head, item );
        shListDelItem( head, del_item, scan_free );
    }

    return( number_items );
}

/*-----------------------------------------------------------------------------
Routine Name: scan_get_by_bssid
Routine Description: Gets scan_result pointer to item by bssid
Arguments:
   mydrv   - pointer to private driver data structure
   bssid   - pointer to bssid value
Return Value: pointer to scan_result item
-----------------------------------------------------------------------------*/
struct wpa_scan_result *scan_get_by_bssid( struct wpa_driver_ti_data *mydrv,
                         u8 *bssid )
{
    SHLIST *head = &(mydrv->scan_merge_list);
    SHLIST *item;
    struct wpa_scan_result *cur_res;

    item = shListGetFirstItem( head );  /* Add/Remove missing items */
    if( item == NULL )
        return( NULL );
    do {
        cur_res =
          (struct wpa_scan_result *)&(((scan_merge_t *)(item->data))->scanres);
        if( (!os_memcmp(cur_res->bssid, bssid, ETH_ALEN)) &&
            (!IS_HIDDEN_AP(cur_res)) ) {
            return( cur_res );
        }
        item = shListGetNextItem( head, item );
    } while( item != NULL );

    return( NULL );
}
