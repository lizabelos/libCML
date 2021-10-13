import statistics


class Table:

    def __init__(self, xabs, yabs):
        self.xabs = xabs
        self.yabs = yabs
        self.width = len(xabs)
        self.height = len(yabs)
        self.xmap = {}
        self.ymap = {}

        for i in range(0, self.width):
            self.xmap[self.xabs[i]] = i

        for i in range(0, self.height):
            self.ymap[self.yabs[i]] = i

        self.data = []
        for y in range(0, self.height):
            line = []
            for x in range(0, self.width):
                line.append(None)
            self.data.append(line)

    def xindex(self, value):
        return self.xmap[value]

    def yindex(self, value):
        return self.ymap[value]

    def set(self, x, y, value):
        self.data[self.yindex(y)][self.xindex(x)] = value

    def get(self, x, y):
        return self.data[self.yindex(y)][self.xindex(x)]


class FileTable:

    def __init__(self, xabs, yabs, filename):
        self.xabs = xabs
        self.yabs = yabs
        self.table = Table(xabs, yabs)
        self.filenamme = filename

    def set(self, x, y, value):
        self.table.set(x, y, value)
        self.save()

    def get(self, x, y):
        return self.table.get(x, y)

    def save(self, separator = "\t"):
        f = open(self.filenamme, "w")
        f.write(separator)
        for value in self.table.xabs:
            f.write(str(value))
            f.write(separator)
        f.write("\n")
        y = 0
        for line in self.table.data:
            f.write(str(self.table.yabs[y]))
            f.write(separator)
            for value in line:
                if value is not None:
                    f.write(str(value))
                    # f.write(str(value).replace(".", ","))
                f.write(separator)
            f.write("\n")
            y = y + 1
        f.close()


class MedianTableProxy:

    def __init__(self, table):
        self.table = table
        self.mediantable = Table(table.xabs, table.yabs)

    def set(self, x, y, value):
        if self.mediantable.get(x, y) is None:
            self.mediantable.set(x, y, [value])
        else:
            self.mediantable.get(x, y).append(value)
        self.table.set(x, y, statistics.median(self.mediantable.get(x, y)))

    def get(self, x, y):
        return self.table.get(x, y)


class SumTableProxy:

    def __init__(self, table):
        self.table = table
        self.mediantable = Table(table.xabs, table.yabs)

    def set(self, x, y, value):
        if self.mediantable.get(x, y) is None:
            self.mediantable.set(x, y, value)
        else:
            self.mediantable.set(value + self.mediantable.get(x, y))
        self.table.set(x, y, self.mediantable.get(x, y))

    def get(self, x, y):
        return self.table.get(x, y)
