//
//  IMUManager.swift
//  Mobile Systems Project 2
//
//  Created by Adam Fishman on 5/7/20.
//  Copyright © 2020 Adam Fishman. All rights reserved.
//

import Foundation
import CoreMotion
import simd

typealias MotionHandler = (CMDeviceMotion) -> Void
let CALIBRATION_LENGTH = 100
let dT = 0.01
let ALPHA: Double = 0.9
let PRINT_INTERVAL = Int(1 / dT)

final class IMUManager : ObservableObject {
    var motionManager: CMMotionManager
    let queue = OperationQueue()
    @Published var tracking = false
    @Published var calibrated = false
    @Published var attitude_string = ""
    var accelerometerBias = (x: -1.0, y: -1.0, z: -1.0)
    var accelerometerNoise = (x: -1.0, y: -1.0, z: -1.0)
    var gyroBias = (x: -1.0, y: -1.0, z: -1.0)
    var gyroNoise = (x: -1.0, y: -1.0, z: -1.0)
    // var attitudeEstimate = simd_quatd(ix: 0, iy: 0, iz: 0, r: 1)
    var attitudeEstimate = simd_quatd(ix: 0.7071067811865475, iy: 0, iz: 0, r: 0.7071067811865476)
    var accelerometerCalibrationData = (
        x: [Double](repeating: -1, count: CALIBRATION_LENGTH),
        y: [Double](repeating: -1, count: CALIBRATION_LENGTH),
        z: [Double](repeating: -1, count: CALIBRATION_LENGTH)
    )
    var gyroCalibrationData = (
        x: [Double](repeating: -1, count: CALIBRATION_LENGTH),
        y: [Double](repeating: -1, count: CALIBRATION_LENGTH),
        z: [Double](repeating: -1, count: CALIBRATION_LENGTH)
    )
    var calibrationTimer = 0
    var printTimer = 0
    
    init() {
        motionManager = CMMotionManager()
        queue.maxConcurrentOperationCount = 1
        queue.name = "IMUEngineQueue"
        accelerometerBias = (-1, -1, -1)
        gyroBias = (-1, -1, -1)
    }
    
    func track(processData: @escaping MotionHandler) {
        if tracking {
            print("Stop tracking before calling track")
        }
        tracking = true
        if !motionManager.isDeviceMotionAvailable {
            print("Unable to access device motion")
            return
        }
        motionManager.deviceMotionUpdateInterval = dT
        motionManager.startDeviceMotionUpdates(to: queue) {
            (deviceMotion: CMDeviceMotion?, error: Error?) in
            if error != nil {
                print("Encountered error: \(error!)")
            }
            
            if deviceMotion != nil {
                processData(deviceMotion!)
            }
        }
    }
    
    func calibrate() {
        self.calibrated = false
        if self.tracking {
            stop()
        }
        usleep(200000)
        track(processData: self.processForCalibration)
    }
    
    func processForCalibration(deviceMotion: CMDeviceMotion) {
        DispatchQueue.main.async {
            let acceleration = (
                x: deviceMotion.userAcceleration.x + deviceMotion.gravity.x,
                y: deviceMotion.userAcceleration.y + deviceMotion.gravity.y,
                z: deviceMotion.userAcceleration.z + deviceMotion.gravity.z
            )
            if !self.calibrated && self.calibrationTimer < CALIBRATION_LENGTH {
                self.accelerometerCalibrationData.x[self.calibrationTimer] = acceleration.x
                self.accelerometerCalibrationData.y[self.calibrationTimer] = acceleration.y
                self.accelerometerCalibrationData.z[self.calibrationTimer] = acceleration.z
                
                self.gyroCalibrationData.x[self.calibrationTimer] = deviceMotion.rotationRate.x
                self.gyroCalibrationData.y[self.calibrationTimer] = deviceMotion.rotationRate.y
                self.gyroCalibrationData.z[self.calibrationTimer] = deviceMotion.rotationRate.z
                
                self.calibrationTimer += 1
            } else if self.calibrationTimer == CALIBRATION_LENGTH {
                self.gyroBias.x = self.mean(arr: self.gyroCalibrationData.x)
                self.gyroBias.y = self.mean(arr: self.gyroCalibrationData.y)
                self.gyroBias.z = self.mean(arr: self.gyroCalibrationData.z)

                self.gyroNoise.x = self.variance(arr: self.gyroCalibrationData.x.map {$0 - self.gyroBias.x})
                self.gyroNoise.y = self.variance(arr: self.gyroCalibrationData.y.map {$0 - self.gyroBias.y})
                self.gyroNoise.z = self.variance(arr: self.gyroCalibrationData.z.map {$0 - self.gyroBias.z})
                print("Gyro Bias (x, y, z): \(self.gyroBias.x), \(self.gyroBias.y), \(self.gyroBias.z)")
                print("Gyro Noise (x, y, z): \(self.gyroNoise.x), \(self.gyroNoise.y), \(self.gyroNoise.z)")
                

                self.accelerometerBias.x = self.mean(arr: self.accelerometerCalibrationData.x)
                self.accelerometerBias.y = self.mean(arr: self.accelerometerCalibrationData.y)
                self.accelerometerBias.z = self.mean(arr: self.accelerometerCalibrationData.z) + 1
                
                self.accelerometerNoise.x = self.variance(arr: self.accelerometerCalibrationData.x.map{$0 - self.accelerometerBias.x})
                self.accelerometerNoise.y = self.variance(arr: self.accelerometerCalibrationData.y.map{$0 - self.accelerometerBias.y})
                self.accelerometerNoise.z = self.variance(arr: self.accelerometerCalibrationData.z.map{$0 - self.accelerometerBias.z})
                print("Accel Bias (x, y, z): \(self.accelerometerBias.x), \(self.accelerometerBias.y), \(self.accelerometerBias.z)")
                print("Accel Noise (x, y, z): \(self.accelerometerNoise.x), \(self.accelerometerNoise.y), \(self.accelerometerNoise.z)")

                self.calibrationTimer = 0
                self.calibrated = true
            } else {
                self.processDeviceMotion(deviceMotion: deviceMotion)
                
            }
        }
    }
    
    func stop() {
        self.tracking = false
        if self.motionManager.isDeviceMotionAvailable {
            self.motionManager.stopDeviceMotionUpdates()
        }
    }
    
    func processDeviceMotion(deviceMotion: CMDeviceMotion) {
        let acceleration = (
            x: deviceMotion.userAcceleration.x + deviceMotion.gravity.x,
            y: deviceMotion.userAcceleration.y + deviceMotion.gravity.y,
            z: deviceMotion.userAcceleration.z + deviceMotion.gravity.z
        )
        // print("Acceleration (x, y, z): \(acceleration.x), \(acceleration.y), \(acceleration.z)")
        updateGyroEstimate(gyroReading: deviceMotion.rotationRate)
        updateAccelerometerEstimate(accelerometerReading: acceleration)
        
        let x = String(format: "%.2f", attitudeEstimate.axis.x)
        let y = String(format: "%.2f", attitudeEstimate.axis.y)
        let z = String(format: "%.2f", attitudeEstimate.axis.z)
        let angle  = String(format: "%.2f", attitudeEstimate.angle)
        self.attitude_string = "Axis: (\(x), \(y), \(z))\n Angle: \(angle)"
        if self.printTimer >= PRINT_INTERVAL {
            print("\(attitudeEstimate.angle)")
            self.printTimer = 0
        }
        self.printTimer += 1
    }
    
    func updateAccelerometerEstimate(accelerometerReading: (x: Double, y: Double, z: Double)) {
        let corrected = (
            x: accelerometerReading.x - accelerometerBias.x,
            y: accelerometerReading.y - accelerometerBias.y,
            z: accelerometerReading.z - accelerometerBias.z
        )
        
        let a = simd_quatd(
            ix: corrected.x,
            iy: corrected.y,
            iz: corrected.z,
            r: 0
        )
        let a_in_gyro_frame = attitudeEstimate * a * attitudeEstimate.conjugate
        let norm = sqrt(pow(a_in_gyro_frame.imag.x, 2) + pow(a_in_gyro_frame.imag.y, 2) + pow(a_in_gyro_frame.imag.z, 2))
        let t = SIMD3<Double>(
            a_in_gyro_frame.imag.z,
            0,
            -a_in_gyro_frame.imag.x
        )
        let phi = acos(a_in_gyro_frame.imag.y / norm)
        attitudeEstimate = simd_quatd(angle: -ALPHA * phi, axis: t) * attitudeEstimate
    }
    
    func updateGyroEstimate(gyroReading: CMRotationRate) {
        let corrected = (
            x: gyroReading.x - gyroBias.x,
            y: gyroReading.y - gyroBias.y,
            z: gyroReading.z - gyroBias.z
        )
        let l: Double = sqrt(pow(corrected.x, 2) + pow(corrected.y, 2) + pow(corrected.z, 2))
        let axis = SIMD3<Double>(x: corrected.x / l, y: corrected.y / l, z: corrected.z / l)
        let theta =  l * dT
        let q = simd_quatd(angle: theta, axis: axis)
        
        attitudeEstimate = attitudeEstimate * q
    }
    
    func mean(arr : [Double]) -> Double
    {
        let length = Double(arr.count)
        return arr.reduce(0, {$0 + $1}) / length
    }
    
    func variance(arr : [Double]) -> Double
    {
        let length = Double(arr.count)
        let avg = arr.reduce(0, {$0 + $1}) / length
        let sumOfSquaredAvgDiff = arr.map { pow($0 - avg, 2.0)}.reduce(0, {$0 + $1})
        return (sumOfSquaredAvgDiff / length)
    }
}
