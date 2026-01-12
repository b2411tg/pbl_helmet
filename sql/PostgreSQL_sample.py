import psycopg2
import pandas as pd
from psycopg2 import sql
from psycopg2.extras import execute_values
import time
import datetime as dt

TABLE_NAME = "helmet_log"
#HOST_IP = "192.168.1.31"    # 接続するIPアドレス
HOST_IP = "localhost"       # 自身に構築したデータベースに接続する場合

''' PostgreSQLクラス '''
class PostgreSQL:  

    def __init__(self, shared):
        self.host_ip = HOST_IP
        self.table_name = TABLE_NAME
        self.shared = shared

    ''' PostgreSQL に接続 '''
    def connect(self):
        con = psycopg2.connect(f"host={self.host_ip} port=5432 dbname=postgres user=postgres password=aiit connect_timeout=5")
        return con

    ''' PostgreSQLデータベースのデータ取得 '''
    def get_data(self, con):
        with con.cursor() as cur:   #ｶｰｿﾙ作成
            con.set_client_encoding('UTF8')
            try:
                cur.execute("SELECT * FROM " + self.table_name)  #SQL実行
                df = pd.DataFrame(cur.fetchall(), columns=[col.name for col in cur.description])
            except Exception as e:
                return e
        con.commit()                #確定
        return df

    ''' PostgreSQLデータベースから期間のデータ削除 '''
    def del_data_span(self, con, start, end):
        with con.cursor() as cur:   #ｶｰｿﾙ作成
            try:
                cur.execute(f"DELETE FROM {self.table_name} WHERE start>='{start}' AND finish<='{end}';")  #SQL実行
            except:
                return False
        con.commit()                #確定
        return True

    ''' PostgreSQLデータベースからデータ数取得 '''
    def get_count(self, con):
        with con.cursor() as cur:   #ｶｰｿﾙ作成
            try:
                cur.execute(f"SELECT COUNT(*) FROM {self.table_name};")  #SQL実行
                rows = cur.fetchall()[0][0]
            except:
                return False 
        con.commit()                #確定
        return rows

    ''' PostgreSQL にデータベース作成 '''
    def create_db(self):
        con = self.connect()
        with con.cursor() as cur:   #ｶｰｿﾙ作成
            try:
                sql = f"CREATE DATABASE helmet_log OWNER postgres;"
                cur.execute(sql)    #SQL実行
            except:
                con.close()
                return
        con.commit()                #確定
        con.close()
        return True

    ''' PostgreSQL にテーブル作成 '''
    def create_table(self):
        con = self.connect()
        with con.cursor() as cur:   #ｶｰｿﾙ作成
            try:
                sql = f"CREATE TABLE {self.table_name} (ts TEXT, Latitude TEXT, Longitude TEXT, status TEXT);"
                cur.execute(sql)    #SQL実行
            except:
                con.close()
                return
        con.commit()                #確定
        con.close()
        return True

    ''' PostgreSQL からテーブル削除 '''
    def remove_table(self):
        con = self.connect()
        with con.cursor() as cur:   #ｶｰｿﾙ作成
            try:
                sql = f"DROP TABLE {self.table_name};"
                cur.execute(sql)    #SQL実行
            except Exception as e:
                return e
        con.commit()                #確定
        con.close()
        return True

    ''' PostgreSQLにデータを追加 '''
    def insert_row(self, con, data):
        data = data.split(",")
        with con.cursor() as cur:   #ｶｰｿﾙ作成
            try:
                sql = f"INSERT INTO {self.table_name}(ts, latitude, longitude, status) VALUES('{data[0]}', '{data[1]}', '{data[2]}', '{data[3]}');"
                cur.execute(sql)
            except Exception as e:
                print(e)
                return False
        con.commit()                #確定
        #print(sql)
        return True

    def main(self):
#        self.create_db()
#        self.remove_table()
#        self.create_table()
        con = self.connect()
        while True:
            self.shared.sql_insert_line.wait()
            data = (self.shared.sql_insert_data).split(",")
            self.shared.sql_insert_line.clear()
            st = f"{data[0]},{data[1]},{data[2]},{self.shared.detect_status}"
            self.insert_row(con, st)
            if self.shared.detect_status == 0 or self.shared.detect_status == 2:
                continue
            else:
                if self.shared.detect_reverse.is_set():
                    self.shared.detect_status = 2   # 逆走中の時はｽﾃｰﾀｽは逆走中に戻す
                else:
                    self.shared.detect_status = 0   # 逆走中以外はｽﾃｰﾀｽｸﾘｱ
