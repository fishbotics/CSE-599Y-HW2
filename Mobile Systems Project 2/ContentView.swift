//
//  ContentView.swift
//  Mobile Systems Project 2
//
//  Created by Adam Fishman on 5/7/20.
//  Copyright © 2020 Adam Fishman. All rights reserved.
//

import SwiftUI

struct ContentView: View {
    @EnvironmentObject var imuManager: IMUManager
    
    var body: some View {
        VStack {
            if imuManager.attitude_string == "" {
                Text("Adam's CSE 599Y Attitude Estimation App")
                    .font(.title)
                    .foregroundColor(Color.white)
            } else {
                Text(imuManager.attitude_string)
                    .font(.title)
                    .foregroundColor(Color.white)
                
            }
            if imuManager.tracking && !imuManager.calibrated {
                Text("Calibrating")
                    .foregroundColor(Color.green)
            } else {
                Button(action: {
                    self.calibrate()
                }) {
                    if imuManager.calibrated {
                        Text("Recalibrate")
                    } else {
                        Text("Calibrate")
                    }
                }
            }
        }
    }
    
    func calibrate() {
        imuManager.calibrate()
    }
}

struct ContentView_Previews: PreviewProvider {
    static var previews: some View {
        ContentView()
            .environmentObject(IMUManager())
    }
}
