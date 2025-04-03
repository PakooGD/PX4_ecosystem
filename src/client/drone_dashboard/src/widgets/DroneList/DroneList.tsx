import React, { useState, useEffect } from 'react';
import { observer } from 'mobx-react-lite';
import { droneStore } from '../../shared/stores/droneStore';
import styles from './DroneList.module.scss';

export const DroneList = observer(() => {
  const [checkboxStates, setCheckboxStates] = useState<{ [key: string]: boolean }>({});
  const [menuVisibility, setMenuVisibility] = useState<{ [key: string]: boolean }>({});
  const [redirectPaths, setRedirectPaths] = useState<{ [key: string]: string }>({});

  // Инициализация состояний чекбоксов на основе данных из store
  useEffect(() => {
    const initialCheckboxStates: { [key: string]: boolean } = {};
    droneStore.drones.forEach(drone => {
      drone.topics.forEach(topic => {
        const key = `${drone.id}-${topic.topic}`;
        initialCheckboxStates[key] = topic.status;
      });
    });
    setCheckboxStates(initialCheckboxStates);
  }, [droneStore.drones]);

  const handleCheckboxChange = (droneId: string, topicName: string, isChecked: boolean) => {
    setCheckboxStates(prev => ({
      ...prev,
      [`${droneId}-${topicName}`]: isChecked,
    }));
  };

  const handleRedirectPathChange = (droneId: string, path: string) => {
    setRedirectPaths(prev => ({
      ...prev,
      [droneId]: path,
    }));
  };

  const toggleMenuVisibility = (droneId: string) => {
    setMenuVisibility(prev => ({
      ...prev,
      [droneId]: !prev[droneId], 
    }));
  };

  const handleUpdateSubscriptions = async (droneId: string) => {
    const selectedPath = redirectPaths[droneId] || 'site'; 
    try {
      await droneStore.redirectLogs(droneId, selectedPath);

      const topics = droneStore.drones.find(drone => drone.id === droneId)?.topics.map(topic => ({
        name: topic.name,
        topic: topic.topic,
        status: checkboxStates[`${droneId}-${topic.topic}`] ?? topic.status,
      }));
      
      if (topics) {
        await droneStore.updateSubscriptions(droneId, topics);
      }
    } catch (error) {
      console.error('Ошибка при обновлении подписок:', error);
    }
  };

  const handleLoadUlog = async (droneId: string) => {
    try {
      await droneStore.loadLogs(droneId);
    } catch (error) {
      console.error('Ошибка при загрузке логов:', error);
    }
  };

  return (
    <div className={styles.droneList}>
      {droneStore.drones.map(drone => (
        <div key={drone.id} className={styles.droneBlock}>
          <div className={styles.droneHeader} onClick={() => toggleMenuVisibility(drone.id)}>
            <div className={styles.droneTitle}>
              <span className={styles.droneId}>{drone.id}</span>
              <span className={`${styles.statusIndicator} ${drone.status === 'online' ? styles.online : styles.offline}`}>
                {drone.status === 'online' ? 'Online' : 'Offline'}
              </span>
              <span className={styles.droneIp}>{drone.ip_address}</span>
            </div>
          </div>
          
          {menuVisibility[drone.id] && (
            <div className={styles.topics}>
              {drone.topics.map(topic => {
                const checkboxKey = `${drone.id}-${topic.topic}`;
                return (
                  <div key={checkboxKey} className={styles.topic}>
                    <label>
                      <input
                        type="checkbox"
                        checked={checkboxStates[checkboxKey] ?? topic.status}
                        onChange={e => {
                          e.stopPropagation();
                          handleCheckboxChange(drone.id, topic.topic, e.target.checked);
                        }}
                      />
                      <span className={styles.topicName}>{topic.name}</span>
                      <span className={styles.schemaName}>{topic.topic}</span>
                    </label>
                  </div>
                );
              })}
            </div>
          )}
          
          <div className={styles.droneMenu}>
            <div className={styles.menuSection}>
              <button
                onClick={e => {
                  e.stopPropagation();
                  handleUpdateSubscriptions(drone.id);
                }}
                className={styles.updateButton}
              >
                Изменить параметры
              </button>
              <div className={styles.redirectSection}>
                <select
                  value={redirectPaths[drone.id] || 'site'}
                  onChange={e => handleRedirectPathChange(drone.id, e.target.value)}
                  className={styles.redirectSelect}
                >
                  <option value="site">Окно логов на сайте</option>
                  <option value="foxglove">Foxglove</option>
                </select>
              </div>
            </div>
            <button
              onClick={e => {
                e.stopPropagation();
                handleLoadUlog(drone.id);
              }}
              className={styles.updateButton}
            >
              Загрузить ULOG
            </button>
          </div>
        </div>
      ))}
    </div>
  );
});