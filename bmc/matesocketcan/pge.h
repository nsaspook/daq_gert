/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/cFiles/file.h to edit this template
 */

/* 
 * File:   pge.h
 * Author: root
 *
 * Created on December 31, 2023, 5:55 PM
 */

#ifndef PGE_H
#define PGE_H

#ifdef __cplusplus
extern "C" {
#endif


#define E_MONTH         2266.0f // Kwh
#define G_MONTH         1000.0f // kWh
#define E_DAYS          31.0f
#define E_PER_DAY       E_MONTH/E_DAYS
#define E_PER_HOUR      E_PER_DAY/24.0f
#define G_PER_DAY       G_MONTH/E_DAYS
#define G_PER_HOUR      G_PER_DAY/24.0f
        
#define PGE_ZERO

#ifdef __cplusplus
}
#endif

#endif /* PGE_H */

