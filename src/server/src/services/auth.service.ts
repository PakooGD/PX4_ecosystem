import { OperationFailed } from '../utils/errors/errors';
import { Drone, Topic } from '../models';
import { Sequelize, Op } from 'sequelize';

require('dotenv').config();

const jwt = require('jsonwebtoken');

export class AuthService {
    public static async Auth(drone_id: string, ip_address: string): Promise<{ accessToken: string, refreshToken: string }> {
        try {
            const transaction = await Drone.sequelize?.transaction();

            const accessToken = jwt.sign({ drone_id }, process.env.SECRET_KEY, { expiresIn: '15m' });
            const refreshToken = jwt.sign({ drone_id }, process.env.REFRESH_SECRET_KEY, { expiresIn: '2d' });
            
            const decoded = jwt.decode(refreshToken);
            const expiresAt = new Date(decoded.exp * 1000);
    
            const [drone, created] = await Drone.findOrCreate({
                where: { drone_id },
                defaults: {drone_id, ip_address, refreshToken, expiresAt, status: 'online'},
                transaction
            });

            if (!created) {
                await drone.update({ ip_address, refreshToken, expiresAt, status: 'online' }, { transaction });
            }

            await transaction?.commit();

            return { accessToken, refreshToken };

        } catch (err) {
            throw new OperationFailed(`Auth failed: ${err}`); 
             
        }
    };

    public static async verifyAuthTokens(req:any): Promise<{ drone_id: string }> {
        try {
            const token = req.headers['authorization']?.split(' ')[1];
            if (!token || token == 'None') {
                throw new Error('Auth tokens missing');
            }
            return jwt.verify(token, process.env.SECRET_KEY) as { drone_id: string };
        } catch (err) {
            throw new OperationFailed(`Access token failed: ${err}`);
        }
    }
    
    public static async Refresh(refreshToken: any): Promise<{ newAccessToken: string, newRefreshToken: string }>  {
        const transaction = await Drone.sequelize?.transaction();
        try {
            const decoded = jwt.verify(refreshToken, process.env.REFRESH_SECRET_KEY);
            const drone_id = decoded.drone_id;

            const drone = await Drone.findOne({
                where: { drone_id },
                transaction
            });

            if (!drone) {
                throw new OperationFailed('Drone not found');
            }

            if (drone.refreshToken !== refreshToken) {
                throw new OperationFailed('Invalid refresh token');
            }

            if (drone.expiresAt < new Date()) {
                throw new OperationFailed('Refresh token expired');
            }

            const newAccessToken = jwt.sign({ drone_id }, process.env.SECRET_KEY, { expiresIn: '15m' });
            const newRefreshToken = jwt.sign({ drone_id }, process.env.REFRESH_SECRET_KEY, { expiresIn: '2d' });

            const newDecoded = jwt.decode(newRefreshToken);
            const newExpiresAt = new Date((newDecoded as any).exp * 1000);

            await drone.update({
                refreshToken: newRefreshToken,
                expiresAt: newExpiresAt,
                status: 'online'
            }, { transaction });

            await transaction?.commit();
            return { newAccessToken, newRefreshToken };
        } catch (err) {
            await transaction?.rollback();
            throw new OperationFailed(`Refresh failed: ${err}`);
        }
    };
    
    public static async HandleLogout(drone_id: string): Promise<void> {
        try {
            await Drone.update(
                { status: 'offline' },
                { where: { drone_id } }
            );
        } catch (err) {
            throw new OperationFailed(`Failed to logout: ${err}`);
        }
    };

    public static async SetOnlineStatus(drone_id: string): Promise<void>  {
        try {
            await Drone.update(
                { status: 'online' },
                { where: { drone_id } }
            );
        } catch (err) {
            throw new OperationFailed(`Failed to signin: ${err}`);
        }
    };

    public static async FetchDrones(): Promise<Array<{
        id: string;
        topics: any[];
        status: string;
        ip_address: string;
    }>> {
        try {
            await Drone.update(
                { status: 'offline' },
                { 
                    where: { 
                        expiresAt: { [Op.lt]: new Date() },
                        status: 'online'
                    }
                }
            );
            const drones = await Drone.findAll({
                include: [{
                    model: Topic,
                    as: 'topics'
                }]
            });

            return drones.map(drone => ({
                id: drone.drone_id,
                topics: drone.topics || [],
                status: drone.status,
                ip_address: drone.ip_address
            }));
        } catch (err) {
            throw new OperationFailed(`Failed to get Ids: ${err}`);
        }
    };

    public static async SetAllDronesOffline(): Promise<void> {
        try {
            await Drone.update(
                { status: 'offline' },
                { where: { status: 'online' } }
            );
        } catch (err) {
            throw new OperationFailed(`Failed to set all drones offline: ${err}`);
        }
    }

    public static async UpdateData(drone_id: string, data: { topics: any[], ip: string }): Promise<void> {
        const transaction = await Drone.sequelize?.transaction();
        try {
            // Update drone IP address
            await Drone.update(
                { ip_address: data.ip },
                { where: { drone_id }, transaction }
            );

            // Update topics
            if (data.topics && data.topics.length > 0) {
                const existingTopics = await Topic.findAll({
                    where: { drone_id },
                    transaction
                });
                const topicsToCreate = [];
                const topicsToUpdate = [];

                for (const newTopic of data.topics) {
                    const existingTopic = existingTopics.find(t => t.topic === newTopic.topic);
                    
                    if (existingTopic) {
                        topicsToUpdate.push(
                            Topic.update(newTopic, {
                                where: { id: existingTopic.id },
                                transaction
                            })
                        );
                    } else {
                        topicsToCreate.push({
                            ...newTopic,
                            drone_id
                        });
                    }
                }
    
                const topicsToRemove = existingTopics.filter(et => 
                    !data.topics.some(t => t.topic === et.topic)
                );
                
                if (topicsToRemove.length > 0) {
                    await Topic.destroy({
                        where: {
                            id: topicsToRemove.map(t => t.id)
                        },
                        transaction
                    });
                }
    
                // Выполняем операции
                await Promise.all(topicsToUpdate);
                
                if (topicsToCreate.length > 0) {
                    await Topic.bulkCreate(topicsToCreate, { transaction });
                }
            }

            await transaction?.commit();
        } catch (err) {
            await transaction?.rollback();
            throw new OperationFailed(`Failed to update drone data: ${err}`);
        }
    };
    
}