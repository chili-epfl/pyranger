from math import exp, sin, cos, pi

def gausspuls(x, sigma = 5):
    """
      1 ***-----------+-------------+------------+-------------+------------++
        +  *          +             +exp(-x**2/40) * (0.5*cos(x)+0.5) ****** +
    0.9 ++  **                                                              ++
        |     *                                                              |
    0.8 ++    *                                                             ++
        |      *                                                             |
    0.7 ++      *                                                           ++
        |       *                                                            |
    0.6 ++       *                                                          ++
    0.5 ++        *                                                         ++
        |         *                                                          |
    0.4 ++         **                         ****                          ++
        |            *                     ***    ****                       |
    0.3 ++           *                   **          ***                    ++
        |             *                **              ***                   |
    0.2 ++             *              **                 ***                ++
        |              **            *                      **               |
    0.1 ++               *        ***                         **            ++
        +             +  ***    *** +            +             +****         +
      0 ++------------+-----*****---+------------+-------------+---***********
        0             2             4            6             8             10
    """
    return exp(-x**2/(2 * sigma ** 2)) * (0.5 * cos(x) + 0.5)


def dromadaire(x):
    """
   1 ++--+-----------+-***------+-----------+-------**-+-----------+-----++
      |   +           +*  **     +           (-0.5*cos(x*4*pi)+0.5) ****** |
      |              **     *                     **    *                  |
      |              *      *                    *       *                 |
  0.8 ++            *        *                   *        *               ++
      |             *         *                  *        *                |
      |            *          *                 *          *               |
  0.6 ++           *           *               *            *             ++
      |           *            *              *             *              *
      |          *              *             *             *             *|
      |          *              *             *              *            *|
  0.4 ++        *                *           *                *           *+
      |         *                 *         *                 *          * |
      |        *                  *         *                 *         *  |
  0.2 *+      *                    *       *                   *        * ++
      |*      *                    *      *                     *      *   |
      |**    *                      **    *                      *    *    |
    0 ++ ****                        *****                        ****    ++
      |   +           +          +           +          +           +      |
      +---+-----------+----------+-----------+----------+-----------+------+
          0          0.2        0.4         0.6        0.8          1
    """
    return (-0.5 * cos(x * 4 * pi) + 0.5)
