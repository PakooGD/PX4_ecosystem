// src/shared/api/api.ts
import axios from 'axios';
import { TopicStatus, DroneData } from '../types/ITypes';
import { Filelike } from "@foxglove/ulog";

const API_URL = 'http://localhost:5000/api'; 

const $api = axios.create({
  withCredentials: true,
  baseURL: API_URL,
})
$api.interceptors.request.use((config:any)=>{
  config.headers.Authorization = `Bearer ${localStorage.getItem('token')}`;
  return config;
})
export default $api;



// Получить список дронов
export const fetchDrones = async (): Promise<DroneData[]> => {
  const response = await $api.get(`${API_URL}/drones`, {withCredentials: true});
  return response.data;
};

export const loadLogs = async (data: { droneId: string }): Promise<any> => {
  try {
    const response = await $api.get(`${API_URL}/log/load`, {
      params: data,
      withCredentials: true,
      responseType: 'arraybuffer', // Указываем, что ожидаем бинарные данные
    });

    // Извлекаем имя файла из заголовка Content-Disposition
    const contentDisposition = response.headers['content-disposition'];
    let filename = `${data.droneId}_logfile.ulg`; // Значение по умолчанию, если имя файла не найдено

    if (contentDisposition && contentDisposition.includes('filename=')) {
      filename = contentDisposition
        .split('filename=')[1]
        .replace(/['"]/g, ''); // Убираем кавычки вокруг имени файла
    }

    // Создаем объект Filelike
    const filelike = new BlobFilelike(response.data);

    return { filelike, filename };
  } catch (error) {
    throw new Error('Failed to load logs');
  }
};

// Обновить подписки
export const updateTopics = async (data: { drone_id: string; topics: TopicStatus[] }): Promise<void> => {
  await $api.post(`${API_URL}/topics/update`, data);
};

// Обновить подписки
export const redirectLogs = async (data: { droneId: string, selectedPath: string }): Promise<void> => {
  await $api.post(`${API_URL}/topics/redirect`, data);
};

export class BlobFilelike implements Filelike {
  private _data: Uint8Array;
  private _size: number;

  constructor(data: ArrayBuffer) {
    this._data = new Uint8Array(data);
    this._size = data.byteLength;
  }

  async open(): Promise<number> {
    // Возвращаем размер файла
    return this._size;
  }

  async read(offset: number, length: number): Promise<Uint8Array> {
    // Возвращаем часть данных из файла
    return this._data.slice(offset, offset + length);
  }

  size(): number {
    // Возвращаем размер файла
    return this._size;
  }
}