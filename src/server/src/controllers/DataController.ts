export class DataController {
    static getData() {
        const timestamp = Date.now();
        const data = {
            topic: "/vehicle_odometry",
            timestamp: timestamp,
            data: {
                position: {
                    x: Math.random() * 10,
                    y: Math.random() * 10,
                    z: Math.random() * 10
                },
                velocity: {
                    vx: Math.random(),
                    vy: Math.random(),
                    vz: 0.0
                }
            }
        };
        return data;
    }
}

