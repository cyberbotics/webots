#include <Python.h>

typedef struct {
  PyObject_HEAD char *arr;
  int len;
} ArrayCuckoo;

static int ArrayCuckoo_init(ArrayCuckoo *self, PyObject *args, PyObject *kwds) {
  self->len = 2;
  self->arr = (char *)malloc(self->len * sizeof(char));
  for (int i = 0; i < self->len; i++) {
    self->arr[i] = i;
  }

  return 0;
}

static PyObject *ArrayCuckoo_str(ArrayCuckoo *self) {
  int pos = 0;
  char *s = malloc(self->len * 10);
  for (int i = 0; i < self->len; i++) {
    pos += sprintf(s + pos, "%d, ", self->arr[i]);
  }

  PyObject *ret = PyUnicode_FromString(s);
  free(s);
  return ret;
}

static int ArrayCuckoo_getbuffer(PyObject *obj, Py_buffer *view, int flags) {
  if (view == NULL) {
    PyErr_SetString(PyExc_ValueError, "NULL view in getbuffer");
    return -1;
  }

  ArrayCuckoo *self = (ArrayCuckoo *)obj;
  view->obj = (PyObject *)self;
  view->buf = (void *)self->arr;
  view->len = self->len * sizeof(char);
  view->readonly = 0;
  view->itemsize = sizeof(char);
  view->format = "B";
  view->ndim = 1;
  view->shape = NULL;
  view->strides = NULL;
  view->suboffsets = NULL;
  view->internal = NULL;

  Py_INCREF(self);
  return 0;
}

static PyBufferProcs ArrayCuckoo_as_buffer = {
  (getbufferproc)ArrayCuckoo_getbuffer,
  (releasebufferproc)0,
};

static PyObject *ArrayCuckoo_setpointer(ArrayCuckoo *self, PyObject *original) {
  // This method copies reference (not data) from the passed Python object which supports a Python buffer protocol.
  // In that way this object cuckoos the original one, except this one exposed as `array.array`.

  Py_buffer buffer;

  if (PyObject_GetBuffer(original, &buffer, PyBUF_WRITABLE) == -1) {
    return NULL;
  }

  self->arr = buffer.buf;
  self->len = buffer.len;

  PyBuffer_Release(&buffer);
  return Py_BuildValue("d", 1);
}

static PyMethodDef ArrayCuckoo_methods[] = {{"setpointer", (PyCFunction)ArrayCuckoo_setpointer, METH_O, "Set memory pointer"},
                                           {NULL, NULL, 0, NULL}};

static PyTypeObject ArrayCuckooType = {
  PyVarObject_HEAD_INIT(NULL, 0) "array.array", /* tp_name */
  sizeof(ArrayCuckoo),                           /* tp_basicsize */
  0,                                            /* tp_itemsize */
  0,                                            /* tp_dealloc */
  0,                                            /* tp_print */
  0,                                            /* tp_getattr */
  0,                                            /* tp_setattr */
  0,                                            /* tp_reserved */
  (reprfunc)ArrayCuckoo_str,                     /* tp_repr */
  0,                                            /* tp_as_number */
  0,                                            /* tp_as_sequence */
  0,                                            /* tp_as_mapping */
  0,                                            /* tp_hash  */
  0,                                            /* tp_call */
  (reprfunc)ArrayCuckoo_str,                     /* tp_str */
  0,                                            /* tp_getattro */
  0,                                            /* tp_setattro */
  &ArrayCuckoo_as_buffer,                        /* tp_as_buffer */
  Py_TPFLAGS_DEFAULT,                           /* tp_flags */
  "ArrayCuckoo object",                          /* tp_doc */
  0,                                            /* tp_traverse */
  0,                                            /* tp_clear */
  0,                                            /* tp_richcompare */
  0,                                            /* tp_weaklistoffset */
  0,                                            /* tp_iter */
  0,                                            /* tp_iternext */
  ArrayCuckoo_methods,                           /* tp_methods */
  0,                                            /* tp_members */
  0,                                            /* tp_getset */
  0,                                            /* tp_base */
  0,                                            /* tp_dict */
  0,                                            /* tp_descr_get */
  0,                                            /* tp_descr_set */
  0,                                            /* tp_dictoffset */
  (initproc)ArrayCuckoo_init,                    /* tp_init */
};

static struct PyModuleDef ArrayCuckoo_module = {PyModuleDef_HEAD_INIT, "arraycuckoo", "Module mytest description", -1,
                                               ArrayCuckoo_methods};

PyMODINIT_FUNC PyInit_arraycuckoo(void) {
  PyObject *module = PyModule_Create(&ArrayCuckoo_module);

  ArrayCuckooType.tp_new = PyType_GenericNew;
  if (PyType_Ready(&ArrayCuckooType) < 0)
    return NULL;
  PyModule_AddObject(module, "array", (PyObject *)&ArrayCuckooType);

  return module;
}
