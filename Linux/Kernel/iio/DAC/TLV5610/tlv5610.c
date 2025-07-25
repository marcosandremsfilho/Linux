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

#define TLV5610_ADDR_SHIFT 12

struct tlv5610_chip_info {
	const struct iio_chan_spec *channels;
	unsigned int num_channels;
};

struct tlv5610_cfg {
  int voltage_ref_mv;
  int max_pos;
};

enum tlv5610_id {
  TLV5610 = 0,
};

static const struct tlv5610_cfg tlv5610_cfg[] = {
  [TLV5610] = { .voltage_ref_mv = 4096, .max_pos = 4096, },
};

struct tlv5610_data {
  struct spi_device* spi;
  const struct tlv5610_cfg* cfg;
  struct mutex lock;
  u8 buf[2] ____cacheline_aligned;
};

#define TLV5610_CHANNEL(chan, name) {				\
	.type = IIO_VOLTAGE,					\
	.channel = (chan),					\
	.indexed = 1,						\
	.output = 1,						\
	.datasheet_name = name,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
}

#define DECLARE_TLV5610_CHANNELS(name) \
const struct iio_chan_spec name ## _channels[] = { \
	TLV5610_CHANNEL(0, "OUTA"), \
	TLV5610_CHANNEL(1, "OUTB"), \
	TLV5610_CHANNEL(2, "OUTC"), \
	TLV5610_CHANNEL(3, "OUTD"), \
	TLV5610_CHANNEL(4, "OUTE"), \
	TLV5610_CHANNEL(5, "OUTF"), \
	TLV5610_CHANNEL(6, "OUTG"), \
	TLV5610_CHANNEL(7, "OUTH"), \
}

static DECLARE_TLV5610_CHANNELS(tlv5610);

static const struct tlv5610_chip_info tlv5610_chip_info[] = {
  [TLV5610] {
    .channels = tlv5610_channels,
    .num_channels = ARRAY_SIZE(tlv5610_channels),
  },
};

static int tlv5610_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
  switch (mask) {
    case IIO_CHAN_INFO_SCALE:
      break;
    default:
      return - EINVAL;
  }

  *val = 1;
  return IIO_VAL_INT;
}

static int tlv5610_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	int err;
	struct tlv5610_data *data = iio_priv(indio_dev);
	int address = chan->channel;

  switch (mask) {
    case IIO_CHAN_INFO_RAW:
      if (val > data->cfg->max_pos || val < 0)
        return -EINVAL;
      break;
    default:
      return -EINVAL;
  }

  mutex_lock(&data->lock);

  data->buf[0] = address << TLV5610_ADDR_SHIFT | (val >> 8);
  data->buf[1] = val & 0xFF;

  err = spi_write(data->spi, data->buf, 2);
  mutex_unlock(&data->lock);

  return err;
}

static const struct iio_info tlv5610_info = {
	.read_raw = tlv5610_read_raw,
	.write_raw = tlv5610_write_raw,
};

static int tlv5610_probe(struct spi_device *spi) {
  int err;
  struct device *dev = &spi->dev;
  unsigned long devid;

  struct tlv5610_data *data;
  const struct tlv5610_chip_info *info;

  struct iio_dev *indio_dev;

  indio_dev = devm_iio_device_alloc(dev, sizeof(*data));

  if(!indio_dev)
    return -ENOMEM;

  data = iio_priv(indio_dev);
  spi_set_drvdata(spi, indio_dev);
  data->spi = spi;
  data->cfg = device_get_match_data(&spi->dev);
  if (!data->cfg) {
    devid = spi_get_device_id(spi)->driver_data;
    data->cfg = &tlv5610_cfg[devid];
  }

  mutex_init(&data->lock);

  info = &tlv5610_chip_info[spi_get_device_id(spi)->driver_data];

  indio_dev->name = spi_get_device_id(spi)->name;
  indio_dev->channels = info->channels;
  indio_dev->num_channels = info->num_channels;
  indio_dev->info = &tlv5610_info;

  dev_info(&spi->dev, ("%s probe", indio_dev->name));

  err = devm_iio_device_register(dev, indio_dev);
  if(err) {
    dev_info(&spi->dev, "Unable to register %s\n", indio_dev->name);

		return err;
  }
  return 0;
}

static const struct spi_device_id tlv5610_id[] = {
	{ "tlv5610" },
	{},
};
MODULE_DEVICE_TABLE(spi, tlv5610_id);

static const struct of_device_id tlv5610_of_match[] = {
	{ .compatible = "ti,tlv5610" },
	{ },
};
MODULE_DEVICE_TABLE(of, tlv5610_of_match);

static struct spi_driver tlv5610_driver = {
	.driver = {
		   .name = "tlv5610",
		   .of_match_table = tlv5610_of_match,
		   },
	.probe = tlv5610_probe,
	.id_table = tlv5610_id,
};
module_spi_driver(tlv5610_driver);

MODULE_AUTHOR("Marcos André M S Filho <eng.marcosandresousa@gmail.com>");
MODULE_DESCRIPTION("Texas Instruments TLV5610 DAC driver");
MODULE_LICENSE("GPL v2");
