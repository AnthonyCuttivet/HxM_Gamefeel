/******************************************************************************
  Copyright (c) 2008-2012 Ryan Juckett
  http://www.ryanjuckett.com/

  This software is provided 'as-is', without any express or implied
  warranty. In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.

  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.

  3. This notice may not be removed or altered from any source
     distribution.
******************************************************************************/

/******************************************************************************
  EDITED FROM ORIGINAL SOURCE FOR C# / UNITY USEAGE
******************************************************************************/

using Unity.Mathematics;
using UnityEngine;

public static class SpringsUtils
{
    public const float EPSILON = 0.0001f;

    //******************************************************************************
    // This function will compute the parameters needed to simulate a damped spring
    // over a given period of time.
    // - An angular frequency is given to control how fast the spring oscillates.
    // - A damping ratio is given to control how fast the motion decays.
    //     damping ratio > 1: over damped
    //     damping ratio = 1: critically damped
    //     damping ratio < 1: under damped
    //******************************************************************************
    public static void CalcDampedSpringMotionParams(ref DampedSpringMotionData data, float dt, float angularFrequency, float dampingRatio)
    {
        var epsilon = SpringsUtils.EPSILON;

        // force values into legal range
        if (dampingRatio < 0.0f) dampingRatio = 0.0f;
        if (angularFrequency < 0.0f) angularFrequency = 0.0f;

        // if there is no angular frequency, the spring will not move and we can
        // return identity
        if (angularFrequency < epsilon)
        {
            data.posPosCoef = 1.0f; data.posVelCoef = 0.0f;
            data.velPosCoef = 0.0f; data.velVelCoef = 1.0f;
            return;
        }

        if (dampingRatio > 1.0f + epsilon)
        {
            // over-damped
            float za = -angularFrequency * dampingRatio;
            float zb = angularFrequency * Mathf.Sqrt(dampingRatio * dampingRatio - 1.0f);
            float z1 = za - zb;
            float z2 = za + zb;

            float e1 = Mathf.Exp(z1 * dt);
            float e2 = Mathf.Exp(z2 * dt);

            float invTwoZb = 1.0f / (2.0f * zb); // = 1 / (z2 - z1)

            float e1_Over_TwoZb = e1 * invTwoZb;
            float e2_Over_TwoZb = e2 * invTwoZb;

            float z1e1_Over_TwoZb = z1 * e1_Over_TwoZb;
            float z2e2_Over_TwoZb = z2 * e2_Over_TwoZb;

            data.posPosCoef = e1_Over_TwoZb * z2 - z2e2_Over_TwoZb + e2;
            data.posVelCoef = -e1_Over_TwoZb + e2_Over_TwoZb;

            data.velPosCoef = (z1e1_Over_TwoZb - z2e2_Over_TwoZb + e2) * z2;
            data.velVelCoef = -z1e1_Over_TwoZb + z2e2_Over_TwoZb;
        }
        else if (dampingRatio < 1.0f - epsilon)
        {
            // under-damped
            float omegaZeta = angularFrequency * dampingRatio;
            float alpha = angularFrequency * Mathf.Sqrt(1.0f - dampingRatio * dampingRatio);

            float expTerm = Mathf.Exp(-omegaZeta * dt);
            float cosTerm = Mathf.Cos(alpha * dt);
            float sinTerm = Mathf.Sin(alpha * dt);

            float invAlpha = 1.0f / alpha;

            float expSin = expTerm * sinTerm;
            float expCos = expTerm * cosTerm;
            float expOmegaZetaSin_Over_Alpha = expTerm * omegaZeta * sinTerm * invAlpha;

            data.posPosCoef = expCos + expOmegaZetaSin_Over_Alpha;
            data.posVelCoef = expSin * invAlpha;

            data.velPosCoef = -expSin * alpha - omegaZeta * expOmegaZetaSin_Over_Alpha;
            data.velVelCoef = expCos - expOmegaZetaSin_Over_Alpha;
        }
        else
        {
            // critically damped
            float expTerm = Mathf.Exp(-angularFrequency * dt);
            float timeExp = dt * expTerm;
            float timeExpFreq = timeExp * angularFrequency;

            data.posPosCoef = timeExpFreq + expTerm;
            data.posVelCoef = timeExp;

            data.velPosCoef = -angularFrequency * timeExpFreq;
            data.velVelCoef = -timeExpFreq + expTerm;
        }
    }

    //******************************************************************************
    // This function will update the supplied position and velocity values over
    // according to the motion parameters.
    //******************************************************************************
    public static void UpdateDampedSpringMotion(
        ref float pos,           // position value to update
        ref float vel,           // velocity value to update
        float equilibriumPos, // position to approach
        in DampedSpringMotionData data)   // motion parameters to use
    {
        float oldPos = pos - equilibriumPos; // update in equilibrium relative space
        float oldVel = vel;

        (pos) = oldPos * data.posPosCoef + oldVel * data.posVelCoef + equilibriumPos;
        (vel) = oldPos * data.velPosCoef + oldVel * data.velVelCoef;
    }

    public static void ComputeSpring(float frequency, float damping, ref DampedSpringMotionData data, ref float pos, ref float vel, float targetPos, float dt)
    {
        SpringsUtils.CalcDampedSpringMotionParams(ref data, dt, frequency, damping);
        SpringsUtils.UpdateDampedSpringMotion(ref pos, ref vel, targetPos, in data);
    }

    public static SpringSettings GetSpringSettingsFromDuration(float duration)
    {
        var epsilon = SpringsUtils.EPSILON;
        var dampingRatio = -math.log(epsilon) / (math.sqrt(math.PI * math.PI + math.log(epsilon) * math.log(epsilon)) * duration);
        var frequency = math.sqrt(1 - dampingRatio * dampingRatio) / (2 * math.PI * duration);

        return new SpringSettings
        {
            Damping = dampingRatio,
            Frequency = frequency,
        };
    }


}

//******************************************************************************
// Cached set of motion parameters that can be used to efficiently update
// multiple springs using the same time step, angular frequency and damping
// ratio.
//******************************************************************************
[System.Serializable]
public struct DampedSpringMotionData
{
    // newPos = posPosCoef*oldPos + posVelCoef*oldVel
    public float posPosCoef, posVelCoef;
    // newVel = velPosCoef*oldPos + velVelCoef*oldVel
    public float velPosCoef, velVelCoef;
};
