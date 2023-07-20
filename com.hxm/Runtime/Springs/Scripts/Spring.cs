using Unity.Mathematics;
using UnityEngine;

[System.Serializable]
public struct Spring
{
    [SerializeField] private DampedSpringMotionData data;
    [SerializeField] private float springPosition;
    [SerializeField] private float springVelocity;
    [SerializeField] private float frequency;
    [SerializeField] private float damping;
    [SerializeField] private float equilibrium;

    public Spring(float frequency, float damping)
    {
        this.data = new DampedSpringMotionData();
        this.springPosition = 0f;
        this.springVelocity = 0f;
        this.frequency = frequency;
        this.damping = damping;
        this.equilibrium = 0f;
    }

    public void SetVelocity(float velocity)
    {
        this.springVelocity = velocity;
    }

    public void SetFrequency(float frequency)
    {
        this.frequency = frequency;
    }

    public void SetDamping(float damping)
    {
        this.damping = damping;
    }

    public void SetEquilibrium(float equilibrium)
    {
        this.equilibrium = equilibrium;
    }

    public void SetFrequencyAndDamping(float frequency, float damping)
    {
        this.frequency = frequency;
        this.damping = damping;
    }

    public void Reset()
    {
        this.data = default;
        this.springPosition = 0f;
        this.springVelocity = 0f;
    }

    public float Evaluate(float dt)
    {
        SpringsUtils.ComputeSpring(this.frequency, this.damping, ref this.data, ref this.springPosition, ref this.springVelocity, this.equilibrium, dt);
        return this.springPosition;
    }

    public bool IsOver()
    {
        return math.abs(this.springVelocity) <= 0.01f;
    }
}

public struct Spring3
{
    public Spring x;
    public Spring y;
    public Spring z;

    public Spring3(float frequency, float damping)
    {
        this.x = new Spring(frequency, damping);
        this.y = new Spring(frequency, damping);
        this.z = new Spring(frequency, damping);
    }

    public void SetFrequency(float frequency)
    {
        this.x.SetFrequency(frequency);
        this.y.SetFrequency(frequency);
        this.z.SetFrequency(frequency);
    }

    public void SetDamping(float damping)
    {
        this.x.SetDamping(damping);
        this.y.SetDamping(damping);
        this.z.SetDamping(damping);
    }

    public void SetEquilibrium(float3 equilibrium)
    {
        this.x.SetEquilibrium(equilibrium.x);
        this.y.SetEquilibrium(equilibrium.y);
        this.z.SetEquilibrium(equilibrium.z);
    }

    public void SetFrequencyAndDamping(float frequency, float damping)
    {
        this.x.SetFrequencyAndDamping(frequency, damping);
        this.y.SetFrequencyAndDamping(frequency, damping);
        this.z.SetFrequencyAndDamping(frequency, damping);
    }

    public float3 Evaluate(float dt)
    {
        return new float3
        {
            x = this.x.Evaluate(dt),
            y = this.y.Evaluate(dt),
            z = this.z.Evaluate(dt),
        };
    }
}

[System.Serializable]
public struct SpringSettings
{
    [Range(0f, 2f)]
    public float Damping;
    [Range(0f, 100f)]
    public float Frequency;
}
