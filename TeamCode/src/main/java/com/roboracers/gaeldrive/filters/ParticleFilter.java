package com.roboracers.gaeldrive.filters;


import org.apache.commons.math3.distribution.ChiSquaredDistribution;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.modules.gaeldrive.LocalizationConstants;
import com.roboracers.gaeldrive.particles.Particle;

import com.roboracers.gaeldrive.sensors.SensorModel;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * A filter that uses Monte Carlo methods to find approximate solutions
 *  to filtering problems in a non-linear state space.
 */
public class ParticleFilter {

    /**
     * Hashmap that stores all the particles in Integer/Particle pairs.
     */
     HashMap<Integer, Particle> Particles = new HashMap<>();

    /**
     * Distribution used in the resampling of particles. TODO: Make degreesOfFreed changeable.
     */
    private ChiSquaredDistribution distribution = new ChiSquaredDistribution(2);

    /**
     * Add a particle to the internal Hashmap.
     * @param particle
     */
    public void add(Particle particle) {
        Particles.put(particle.getId(), particle);
    }

    /**
     * Clear all particles.
     */
    public void clear() {
        Particles.clear();
    }

    /**
     * Return the Hashmap that stores the particles.
     * @return Hashmap of particles
     */
    public HashMap<Integer, Particle> getParticles() {
        return this.Particles;
    }


    /**
     * Translate every particle
     * @param translationVector The translation vector that the other particles will be translated by.
     */
    public void translateParticles (RealVector translationVector) {
        // For every particle in our set of Particles
        for (Map.Entry<Integer,Particle> particle2dEntry : Particles.entrySet()) {
            // Get the Particle
            Particle translatedParticle = particle2dEntry.getValue();
            // Add our translational vector
            translatedParticle.setState(translatedParticle.getState().add(translationVector));
            // Set the value as our updated particle
            particle2dEntry.setValue(translatedParticle);
        }
    }

    /**
     * Weigh all the particles in our state space given a set of sensor models.
     * We compared the delta between the actual sensor reading and the
     * sensor value simulated for each one of our sensors.
     * @param models List of models to be used.
     */
    public void weighParticles(List<SensorModel> models) {


        // For every particle in our state space
        for (Map.Entry<Integer, Particle> entry: Particles.entrySet()) {
            // Get the particle from the entry
            Particle particle = entry.getValue();

            double cumalativeWeight = 0;
            double cumalativeWeightModifer = 0;

            // For every sensor model that we are considering
            for (SensorModel model: models) {

                // Get both the actual and simulated reading
                RealVector simulatedSensorValue = model.getSimulatedReading(particle.getState());
                RealVector actualSensorValue = model.getActualReading();

                if (LocalizationConstants.TESTING) {
                    System.out.println("Real Sensor Value: "  + actualSensorValue);
                    System.out.println("Simulated (Random) Sensor Value: " + simulatedSensorValue);
                }

                // Get the delta between our simulated and actual sensor values
                RealVector readingDelta = actualSensorValue.subtract(simulatedSensorValue);
                // Plug the normalized (Euclidean Distance) of the delta into our Chi2 distribution.
                double probSensorGivenState = distribution.density(readingDelta.getNorm());

                // Add the probability multiplied by the weight of the model.
                cumalativeWeight += probSensorGivenState * model.getWeightModifier();
                // Add the weight of this sensor model to the overall weight modifier
                cumalativeWeightModifer += model.getWeightModifier();

            }

            // Calculate the average weights of all the sensors and assign it to the particle
            particle.setWeight(cumalativeWeight/cumalativeWeightModifer);

            if (LocalizationConstants.TESTING) {
                System.out.println("Likeness: " + particle.getWeight() + ", ID: " + particle.getId());
            }
            // Add the particle with the updated weight back into our particle set.
            add(particle);
        }
    }

    public static void resampleParticles() {

    }

    /**
     * Gets the particle with the highest weight.
     * @return Particle of the highest weighted particle.
     */
    public Particle getBestParticle () {

        double highestWeight = 0;
        Integer bestParticleKey = 0;

        // Loop through all weights and get highest weight
        for (Map.Entry<Integer, Particle> particle2dEntry : Particles.entrySet()) {
            double particleWeight = particle2dEntry.getValue().getWeight();
            if (particleWeight > highestWeight) {
                bestParticleKey = particle2dEntry.getKey();
                highestWeight = particleWeight;
            }

        }

        return Particles.get(bestParticleKey);

    }
}
