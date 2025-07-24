#include <linux/cache.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/iio/iio.h>
#include <linux/iio/types.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/spi/spi.h>

#define MCP4361_WRITE (0x00 << 2)
#define MCP4361_READ (0x03 << 2)

#define MCP4361_WIPER_SHIFT 4

#define MCP4361_CMDERR(r) ((r[0]) & 0x02)
#define MCP4361_RAW(r) ((r[0]) == 0xff ? 0x100 : (r[1]))

struct mcp4361_cfg {
  int wipers;
  int max_pos;
  int kohms;
};

struct mcp4361_chip_info {
  const struct iio_chan_spec *channels;
	unsigned int num_channels;
};

enum mcp4361_type {
  MCP4361 = 0,
  MCP4262,
};

static const struct mcp4361_cfg mcp4361_cfg[] = {
	[MCP4361] = {.wipers = 4, .max_pos = 257, .kohms = 50, },
	[MCP4262] = {.wipers = 2, .max_pos = 257, .kohms = 50, },
};

struct mcp4361_data {
  struct spi_device* spi;
  const struct mcp4361_cfg* cfg;
  struct mutex lock;
  u8 buf[2] ____cacheline_aligned;
};

#define MCP4361_CHANNEL(ch){ \
    .type = IIO_RESISTANCE,  \
    .indexed = 1,            \
    .output = 1,             \
    .channel = (ch),         \
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_PROCESSED), \
    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
  }

#define DECLARE_MCP4361_CHANNELS(name) \
const struct iio_chan_spec name ## _channels[] = { \
    MCP4361_CHANNEL(0), \
    MCP4361_CHANNEL(1), \
    MCP4361_CHANNEL(2), \
    MCP4361_CHANNEL(3), \
}

#define DECLARE_MCP4262_CHANNELS(name) \
const struct iio_chan_spec name ## _channels[] = { \
    MCP4361_CHANNEL(0), \
    MCP4361_CHANNEL(1), \
}

static DECLARE_MCP4361_CHANNELS(mcp4361);
static DECLARE_MCP4262_CHANNELS(mcp4262);

static const struct mcp4361_chip_info mcp4361_chip_info[] = {
  [MCP4361] = {
    .channels = mcp4361_channels,
    .num_channels = ARRAY_SIZE(mcp4361_channels),
  },
  [MCP4262] = {
    .channels = mcp4262_channels,
    .num_channels = ARRAY_SIZE(mcp4262_channels),
  },
};

int get_digipot_vol_register(int wiper) {
    if (wiper == 0) {
      return 0;
    } else if (wiper == 1) {
      return 1;
    } else if (wiper == 2) {
      return 6;
    } else if (wiper == 3) {
      return 7;
    } else {
      return -999;
    }
}

int get_digipot_nvol_register(int wiper) {
    if(wiper == 0){
      return 2;
    } else if (wiper == 1) {
      return 3;
    } else if (wiper == 2) {
      return 8;
    } else if (wiper == 3) {
      return 9;
    } else {
      return -999;
  }
}

static int mcp4361_read(struct spi_device *spi, void *buf, size_t len){
	struct spi_transfer t = {
		.tx_buf = buf,
		.rx_buf	= buf,
		.len = len,
	};
	struct spi_message m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return spi_sync(spi, &m);
}

static int mcp4361_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	int err;
	struct mcp4361_data *data = iio_priv(indio_dev);
	int wiper = chan->channel;
  int address = 0;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&data->lock);

    address = get_digipot_vol_register(wiper);
    if (address == -999){
	    mutex_unlock(&data->lock);
      return  -EINVAL;
    }

		data->buf[0] = (address << MCP4361_WIPER_SHIFT) | MCP4361_READ;
		data->buf[1] = 0;

		err = mcp4361_read(data->spi, data->buf, 2);
		if (err) {
			mutex_unlock(&data->lock);
			return err;
		}

		if (!MCP4361_CMDERR(data->buf)) {
			mutex_unlock(&data->lock);
			return -EIO;
		}

		*val = MCP4361_RAW(data->buf);
		mutex_unlock(&data->lock);

		return IIO_VAL_INT;

  case IIO_CHAN_INFO_PROCESSED:
		mutex_lock(&data->lock);

    address = get_digipot_nvol_register(wiper);

    if (address == -999) {
      mutex_unlock(&data->lock);
      return  -EINVAL;
    }

		data->buf[0] = (address << MCP4361_WIPER_SHIFT) | MCP4361_READ;
		data->buf[1] = 0;

		err = mcp4361_read(data->spi, data->buf, 2);
		if (err) {
			mutex_unlock(&data->lock);
			return err;
		}

		if (!MCP4361_CMDERR(data->buf)) {
			mutex_unlock(&data->lock);
			return -EIO;
		}

		*val = MCP4361_RAW(data->buf);
		mutex_unlock(&data->lock);

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 1000 * data->cfg->kohms;
		*val2 = data->cfg->max_pos;
		return IIO_VAL_FRACTIONAL;
	}
	return -EINVAL;
}

static int mcp4361_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	int err;
	struct mcp4361_data *data = iio_priv(indio_dev);
	int wiper = chan->channel;
  int address = -999;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
    address = get_digipot_vol_register(wiper) << MCP4361_WIPER_SHIFT;

		if (val > data->cfg->max_pos || val < 0)
			return -EINVAL;
		break;

  case IIO_CHAN_INFO_PROCESSED:
    address = get_digipot_nvol_register(wiper) << MCP4361_WIPER_SHIFT;

		if (val > data->cfg->max_pos || val < 0)
			return -EINVAL;
		break;

	default:
		return -EINVAL;
	}
  if (address == -999) {
    return -EINVAL;
  }
	mutex_lock(&data->lock);

	data->buf[0] = address << MCP4361_WIPER_SHIFT;
	data->buf[0] |= MCP4361_WRITE | (val >> 8);

	data->buf[1] = val & 0xFF;

	err = spi_write(data->spi, data->buf, 2);
	mutex_unlock(&data->lock);

	return err;
}

static const struct iio_info mcp4361_info = {
	.read_raw = mcp4361_read_raw,
	.write_raw = mcp4361_write_raw,
};

static int mcp4361_probe(struct spi_device *spi)
{
	int err;
	struct device *dev = &spi->dev;
	unsigned long devid;

	struct mcp4361_data *data;
	struct iio_dev *indio_dev;

  const char *label;

  const struct mcp4361_chip_info *info;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);

	spi_set_drvdata(spi, indio_dev);
	data->spi = spi;
	data->cfg = device_get_match_data(&spi->dev);

  devid = spi_get_device_id(spi)->driver_data;
	if (!data->cfg) {
		data->cfg = &mcp4361_cfg[devid];
	}

	mutex_init(&data->lock);

  info = &mcp4361_chip_info[devid];

	indio_dev->name = spi_get_device_id(spi)->name;

  if(of_property_read_bool(dev->of_node, "label")) {
    indio_dev->label = of_property_read_string(dev->of_node, "label", &label);
  }

  indio_dev->channels = info->channels;
	indio_dev->num_channels = info->num_channels;
	indio_dev->info = &mcp4361_info;

  dev_info(&spi->dev, ("Device name: %s probe", indio_dev->name));

	err = devm_iio_device_register(dev, indio_dev);
	if (err) {
		dev_info(&spi->dev, "Unable to register %s\n", indio_dev->name);
		return err;
	}
	return 0;
}

static const struct of_device_id mcp4361_dt_ids[] = {
  { .compatible = "microchip,mcp4361",
		.data = &mcp4361_cfg[MCP4361] },
  { .compatible = "microchip,mcp4262",
    .data = &mcp4361_cfg[MCP4262] },
  {},
};
MODULE_DEVICE_TABLE(of, mcp4361_dt_ids);

static const struct spi_device_id mcp4361_id[] = {
  { "mcp4361", MCP4361 },
  { "mcp4262", MCP4262 },
	{},
};
MODULE_DEVICE_TABLE(spi, mcp4361_id);

static struct spi_driver mcp4361_driver = {
	.driver = {
		.name	= "mcp4361",
		.of_match_table = mcp4361_dt_ids,
	},
	.probe		= mcp4361_probe,
	.id_table	= mcp4361_id,
};
module_spi_driver(mcp4361_driver);

MODULE_AUTHOR("Marcos André M S Filho <eng.marcosandresousa@gmail.com>");
MODULE_DESCRIPTION("MCP4361 digital potentiometer");
MODULE_LICENSE("GPL v2");

