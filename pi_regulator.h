#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

//start the PI regulator thread
void pi_regulator_start(void);
#define DIST_REF 0.07 //[m]

#define VALEUR_MINIMUM 1500

#define VALEUR_DETECTION_CHOC 100
#endif /* PI_REGULATOR_H */
