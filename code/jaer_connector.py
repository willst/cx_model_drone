import sqlite3
import numpy as np
from numpy import linalg as LA
import pandas as pd

class JaerFilter:

    def __init__(self):
        self.conn = sqlite3.connect("/home/willst/test.db")

    def generate_filters(self, angle_range=(130.0,84.0), fh=128, fw=128):
        ''' Generate match filter for optical flow computation, one for left 45 degree
            one for right 45 degree
        '''
        # filter for speed retrieval
        vertical_views = (np.arange(fh, dtype=float) - fh / 2) / fh * (angle_range[1] / 180.0 * np.pi)
        horizontal_views = (np.arange(fw, dtype=float) - fw / 2) / fw * (angle_range[0] / 180.0 * np.pi)
        D = np.ones([fh, fw, 3]) * -1
        D[:, :, 1] = np.tan(vertical_views).reshape(fh, 1)
        D[:, :, 0] = np.tan(horizontal_views)
        sin_theta = LA.norm(D[:, :, 0:2], axis=2) + 0.0000001
        mag_temp = LA.norm(D, axis=2) + 0.0000001
        D /= mag_temp.reshape(fh, fw, 1)
        a_l = np.array([1 / np.sqrt(2), -1 / np.sqrt(2), 0])
        a_r = np.array([1 / np.sqrt(2), 1 / np.sqrt(2), 0])
        left_filter = np.cross(np.cross(D, a_l), D)[:, :, 0:2] / sin_theta.reshape(fh, fw, 1)
        right_filter = np.cross(np.cross(D, a_r), D)[:, :, 0:2] / sin_theta.reshape(fh, fw, 1)

        self._save_filter(left_filter[:,:,0], "left_filter_0")
        self._save_filter(left_filter[:,:,1], "left_filter_1")
        self._save_filter(right_filter[:,:,0], "right_filter_0")
        self._save_filter(right_filter[:,:,1], "right_filter_1")
        return left_filter, right_filter


    def _save_filter(self, filter_array, table_name):
        np.savetxt("{}.csv".format(table_name), filter_array, delimiter=",")
        # cur = self.conn.cursor()
        # cur.execute("DROP TABLE IF EXISTS {}".format(table_name))
        # cur.execute("CREATE TABLE {} (id INTEGER PRIMARY KEY, array BLOB)".format(table_name))
        # cur.execute("INSERT INTO {} VALUES (?,?)".format(table_name), (None, json.dumps(filter_array.tolist())))
        # self.conn.commit()

    # def get_fiter(self, table_name):
    #     cur = self.conn.cursor()
    #     cur.execute("SELECT * FROM {}".format(table_name))
    #     data = cur.fetchall()
    #     print data
    #     return data

class JaerConnector:

    def __init__(self, db_path="/home/willst/test.db", table_name="matchedFilterResults"):
        self.conn = sqlite3.connect(db_path)
        self.table_name = table_name
        cursor = self.conn.execute('SELECT * FROM %s LIMIT 1' % self.table_name)
        self.names = [description[0] for description in cursor.description]
        self.last_entry = self._get_last_entry()

    def query_results(self, last_timestamp=None):
        # type: () -> pd.DataFrame
        ##### type: () -> List[MatchedFilterResult]
        if last_timestamp is None:
            last_timestamp = self.last_entry.timestamp
            print last_timestamp
        sql_statement = 'SELECT * FROM %s WHERE timestamp > %d ORDER BY timestamp DESC' % (self.table_name, last_timestamp)
        return pd.read_sql_query(sql_statement, self.conn)

        # cursor = self.conn.execute(sql_statement)
        # return [self._convert_to_result(row) for row in cursor]

    def _get_last_entry(self):
        # type: () -> MatchedFilterResult
        cursor = self.conn.execute('SELECT * FROM %s ORDER BY timestamp DESC LIMIT 1' % self.table_name)
        return self._convert_to_result(cursor.next())

    def _convert_to_result(self, row):
        # type: (object) -> MatchedFilterResult
        row_dict = dict(zip(self.names, row))
        return MatchedFilterResult(**row_dict)


class MatchedFilterResult:

    def __init__(self, timestamp, leftVX, eventCount, leftVY, duration, rightVX, rightVY):
        self.timestamp = timestamp
        self.leftVX = leftVX
        self.eventCount = eventCount
        self.leftVY = leftVY
        self.duration = duration
        self.rightVX = rightVX
        self.rightVY = rightVY


if __name__ == "__main__":
    jc= JaerConnector()
    df = jc.query_results(0.0).sort_values(by="timestamp")
    # df.to_csv("/home/willst/DVSRecordings/jear_export_3.csv")