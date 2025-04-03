import React, { useState, useEffect, useRef } from 'react';
import { observer } from 'mobx-react-lite';
import { DroneList, Sidebar } from '../../widgets';
import styles from './DashboardPage.module.css';
import { LogViewer } from '../../widgets/LogViewer/LogViewer';


export const DashboardPage = observer(() => {
  const [leftWidth, setLeftWidth] = useState<number>(40); // Начальная ширина левой секции в процентах
  const [isDragging, setIsDragging] = useState<boolean>(false);
  const containerRef = useRef<HTMLDivElement>(null);

  // Обработчик начала перетаскивания
  const handleMouseDown = (e: React.MouseEvent) => {
    e.preventDefault();
    setIsDragging(true);
  };

  // Обработчик перемещения мыши
  const handleMouseMove = (e: MouseEvent) => {
    if (isDragging && containerRef.current) {
      const containerRect = containerRef.current.getBoundingClientRect();
      const newLeftWidth = ((e.clientX - containerRect.left) / containerRect.width) * 100;
      setLeftWidth(Math.min(Math.max(newLeftWidth, 20), 80)); // Ограничиваем ширину от 20% до 80%
    }
  };

  // Обработчик окончания перетаскивания
  const handleMouseUp = () => {
    setIsDragging(false);
  };

  // Добавляем обработчики событий мыши
  useEffect(() => {
    document.addEventListener('mousemove', handleMouseMove);
    document.addEventListener('mouseup', handleMouseUp);

    return () => {
      document.removeEventListener('mousemove', handleMouseMove);
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, [isDragging]);

  return (
    <div className={styles.dashboard}>
      <Sidebar />
      <div className={styles.mainContent} ref={containerRef}>
        <div
          className={styles.droneSection}
          style={{ width: `${leftWidth}%` }}
        >
          <DroneList />
        </div>
        <div
          className={styles.resizer}
          onMouseDown={handleMouseDown}
        />
        <div
          className={styles.logSection}
          style={{ width: `${100 - leftWidth}%` }}
        >
          <LogViewer />
        </div>
      </div>
    </div>
  );
});